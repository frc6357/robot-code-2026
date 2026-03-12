package frc.robot.subsystems.launcher.mechanisms;

import static frc.robot.Konstants.LauncherConstants.Slot0;
import static frc.robot.Konstants.LauncherConstants.Slot1;
import static frc.robot.Konstants.LauncherConstants.kLauncherA;
import static frc.robot.Konstants.LauncherConstants.kLauncherV;
import static frc.robot.Konstants.LauncherConstants.kLauncherS;
import static frc.robot.Konstants.LauncherConstants.kWheelRadius;
import static frc.robot.Konstants.LauncherConstants.kShooterTolerance;
import static frc.robot.Konstants.LauncherConstants.kCoastLauncherRPS;
import static frc.robot.Konstants.LauncherConstants.kStopLauncher;
import static frc.robot.Konstants.LauncherConstants.kUnJamLauncherPauseTime;
import static frc.robot.Konstants.LauncherConstants.kUnJamLauncherRPS;
import static frc.robot.Konstants.LauncherConstants.kUnJamLauncherRunTime;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.PathplannerSubsystem;
import frc.robot.subsystems.launcher.moveandshoot.LauncherTuning;
import lombok.Getter;

import static frc.robot.Ports.LauncherPorts.kLauncherFrontRollers;
import static frc.robot.Ports.LauncherPorts.kLauncherBackRollers;

import org.littletonrobotics.junction.Logger;

public class SK26Launcher extends SubsystemBase implements PathplannerSubsystem {

    @Getter
    private LauncherTuning tuning = new LauncherTuning("SK26Launcher");

    //initialize launcher motor
    TalonFX launchermotor;
    TalonFX launchermotorFollower;

    private final VelocityDutyCycle launcherVelocityControl = new VelocityDutyCycle(0);
    private double targetMotorRPS;
    private String launcherMotorStatus = "Stopped";
    
    public SK26Launcher() {

        //defines motor objects
        launchermotor = new TalonFX(kLauncherFrontRollers.ID);
        launchermotorFollower = new TalonFX(kLauncherBackRollers.ID);
        launchermotorFollower.setControl(new Follower(launchermotor.getDeviceID(), MotorAlignmentValue.Opposed));

        //configures motors
        configMotor(launchermotor);
        configMotor(launchermotorFollower);
    }

    public void configMotor(TalonFX motor) {

        //configures PID values onto the launcher motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        //First slot configuration
        config.withSlot0(
            new Slot0Configs()
                .withKV(kLauncherV).withKA(kLauncherA).withKS(kLauncherS)
                .withKP(Slot0.kP).withKI(Slot0.kI).withKD(Slot0.kD)
        );

        //Second slot configuration
        config.withSlot1(
            new Slot1Configs()
                .withKV(kLauncherV).withKA(kLauncherA).withKS(kLauncherS)
                .withKP(Slot1.kP).withKI(Slot1.kI).withKD(Slot1.kD)
        );

        motor.getConfigurator().apply(config);
        motor.setNeutralMode(NeutralModeValue.Brake); //Makes motor brake when no power is applied
    }

    //Runs the launcher motor to a certain target launch velocity (m/s)
    public void runLauncherExitVel(double targetLaunchVelocity, String launcherMotorStatus) {

        double motorRPS = targetLaunchVelocity/(2*Math.PI*kWheelRadius); //Converts m/s to RPS
        targetMotorRPS = motorRPS;
        launcherVelocityControl.Velocity = motorRPS; 
        launchermotor.setControl(launcherVelocityControl); //makes launcher launch
        this.launcherMotorStatus = launcherMotorStatus;
    }

    //Runs the launcher motor to a certain target motor RPS
    public void runLauncherRPS(double targetMotorRPS, String launcherMotorStatus) {

        this.targetMotorRPS = targetMotorRPS;
        launcherVelocityControl.Velocity = targetMotorRPS;
        launchermotor.setControl(launcherVelocityControl);
        this.launcherMotorStatus = launcherMotorStatus;
    }

    //Debugging stuffs
    //Starts the launcher motor to certain target launch velocity
    /*public void startLauncher(double targetLaunchVelocity) {
        this.targetLaunchVelocity = targetLaunchVelocity;
        launchermotor.set(targetLaunchVelocity);
        isShooting = true;
    }*/
    
    //unjams the motor to allow proper shooting
    public void unJamLauncher(double motorRPS) {
        runLauncherRPS(motorRPS, "Unjamming");
    }

    //checks whether or not the motor is at the target speed
    public boolean isLauncherAtSpeed() {

        double motorRPS = launchermotor.getVelocity().getValueAsDouble();
        return Math.abs(motorRPS - targetMotorRPS) < kShooterTolerance; //Checks if the motor RPS is within the tolerance of the target RPS
    }

    //Sets the motor speed to zero
    public void stopLauncher() {

        launchermotor.set(kStopLauncher);
        targetMotorRPS = kStopLauncher;
        launcherMotorStatus = "Stopped";
    }

    //Slows down the launcher to a very low speed while waiting to shoot
    public void coastLauncher() {

        targetMotorRPS = kCoastLauncherRPS;
        launcherVelocityControl.Velocity = targetMotorRPS;
        launchermotor.setControl(launcherVelocityControl);
        launcherMotorStatus = "Waiting to Shoot";
    }

    // ==================== Inline Command Factories ====================

    /**
     * Returns a command that runs the launcher at the given motor RPS
     * and coasts when ended. Replaces RunLauncherWithRPSCommand.
     *
     * @param targetMotorRPS The target motor speed in rotations per second.
     * @return A command requiring this subsystem.
     */
    public Command runAtRPSCommand(double targetMotorRPS) {
        return this.runEnd(
            () -> runLauncherRPS(targetMotorRPS, "Launching"),
            this::coastLauncher
        );
    }

    /**
     * Returns a command that runs the launcher at the given exit velocity (m/s)
     * and coasts when ended. Replaces RunLauncherWithVelCommand.
     *
     * @param targetLaunchVelocity The target ball exit velocity in m/s.
     * @return A command requiring this subsystem.
     */
    public Command runAtExitVelCommand(double targetLaunchVelocity) {
        return this.runEnd(
            () -> runLauncherExitVel(targetLaunchVelocity, "Launching"),
            this::coastLauncher
        );
    }

    /**
     * Returns a command that oscillates the launcher in reverse and forward
     * to unjam stuck game pieces. Repeats until interrupted, then stops.
     * Replaces the standalone LauncherUnJamCommandGroup.
     *
     * @return A command requiring this subsystem.
     */
    public Command unjamCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> unJamLauncher(-kUnJamLauncherRPS), this),
            Commands.waitSeconds(kUnJamLauncherRunTime),
            Commands.runOnce(() -> unJamLauncher(kStopLauncher), this),
            Commands.waitSeconds(kUnJamLauncherPauseTime),
            Commands.runOnce(() -> unJamLauncher(kUnJamLauncherRPS), this),
            Commands.waitSeconds(kUnJamLauncherRunTime),
            Commands.runOnce(() -> unJamLauncher(kStopLauncher), this),
            Commands.waitSeconds(kUnJamLauncherPauseTime)
        ).repeatedly()
         .finallyDo(() -> stopLauncher())
         .withName("LauncherUnjam");
    }

   
    @Override
    public void periodic() {
        telemeterize();
    }

    public void telemeterize() {
        Logger.recordOutput("Launcher/Status", launcherMotorStatus);
        Logger.recordOutput("Launcher/Is at speed", isLauncherAtSpeed());
        Logger.recordOutput("Launcher/Tangential Velocity", launchermotor.getVelocity().getValueAsDouble()*(2*Math.PI*kWheelRadius));
        Logger.recordOutput("Launcher/RPS", launchermotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Launcher/Target RPS", targetMotorRPS);
    }

    @Override
    public void addPathPlannerCommands() {
        // TODO: Implement PathPlanner commands for launcher
        throw new UnsupportedOperationException("Unimplemented method 'addPathPlannerCommands'");
    }
}
