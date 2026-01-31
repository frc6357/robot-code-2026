package frc.robot.subsystems;

import static frc.robot.Konstants.LauncherConstants.Slot0;
import static frc.robot.Konstants.LauncherConstants.Slot1;
import static frc.robot.Konstants.LauncherConstants.kLauncherA;
import static frc.robot.Konstants.LauncherConstants.kLauncherV;
import static frc.robot.Konstants.LauncherConstants.kLauncherS;
import static frc.robot.Konstants.LauncherConstants.kWheelRadius;
import static frc.robot.Konstants.LauncherConstants.kShooterTolerance;
import static frc.robot.Konstants.LauncherConstants.kIdleLauncherRPS;
import static frc.robot.Konstants.LauncherConstants.kStopLauncher;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Konstants.LauncherConstants.Slot0;
import frc.robot.Konstants.LauncherConstants.Slot1;

import static frc.robot.Ports.LauncherPorts.kFixedLauncherMotor;
import static frc.robot.Ports.LauncherPorts.kFixedLauncherMotorFollower;

/**
 * Launcher subsystem that controls a two-motor flywheel shooter using CTRE TalonFX controllers.
 *
 * <p>The primary TalonFX runs closed-loop velocity control (RPS). A second TalonFX is configured
 * as a follower in the opposed direction to drive the other wheel.
 *
 * <p>Key features:
 * <ul>
 *   <li>Velocity control via {@link com.ctre.phoenix6.controls.VelocityDutyCycle}</li>
 *   <li>Two PID/FF config slots (Slot0 and Slot1) applied to each motor</li>
 *   <li>Conversion helpers for commanding by exit velocity (m/s) or motor speed (RPS)</li>
 *   <li>Telemetry via WPILib {@link edu.wpi.first.util.sendable.Sendable} / SmartDashboard</li>
 * </ul>
 */

public class SK26Launcher extends SubsystemBase {

    //initialize launcher motor
    TalonFX launchermotor;
    TalonFX launchermotorFollower;

    private final VelocityDutyCycle launcherVelocityControl = new VelocityDutyCycle(0);
    private double targetMotorRPS;
    private String launcherMotorStatus = "Stopped";
    
    public SK26Launcher() {

        //defines motor objects
        launchermotor = new TalonFX(kFixedLauncherMotor.ID);
        launchermotorFollower = new TalonFX(kFixedLauncherMotorFollower.ID);

        //configures motors
        configMotor(launchermotor);
        configMotor(launchermotorFollower);

        //sets the follower motor to follow the main launcher motor in opposed direction
        launchermotorFollower.setControl(new Follower(launchermotor.getDeviceID(), MotorAlignmentValue.Opposed));

    }

    /**
     * Configures the TalonFX motor with the appropriate PID values
     * @param motor The TalonFX motor to be configured
     */
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

    /**
     * Runs the launcher motor to a certain target launch velocity (m/s)
     * @param targetLaunchVelocity The target launch velocity in m/s
     * @param launcherMotorStatus The status of the launcher motor (for Smart Dashboard)
     */
    public void runLauncherExitVel(double targetLaunchVelocity, String launcherMotorStatus) {

        double motorRPS = targetLaunchVelocity/(2*Math.PI*kWheelRadius); //Converts m/s to RPS
        targetMotorRPS = motorRPS;
        launcherVelocityControl.Velocity = motorRPS; 
        launchermotor.setControl(launcherVelocityControl); //makes launcher launch
        this.launcherMotorStatus = launcherMotorStatus;
    }

    /**
     * Runs the launcher motor to a certain target motor RPS
     * @param targetMotorRPS The target motor RPS
     * @param launcherMotorStatus The status of the launcher motor (for Smart Dashboard)
     */
    public void runLauncherRPS(double targetMotorRPS, String launcherMotorStatus) {

        this.targetMotorRPS = targetMotorRPS;
        launcherVelocityControl.Velocity = targetMotorRPS;
        launchermotor.setControl(launcherVelocityControl);
        this.launcherMotorStatus = launcherMotorStatus;
    }
    
    /**
     * Unjams the launcher motor by running it at a specified RPS
     * @param motorRPS The motor RPS to unjam the launcher
     */
    public void unJamLauncher(double motorRPS) {
        runLauncherRPS(motorRPS, "Unjamming");
    }

    //checks whether or not the motor is at the target speed
    /**
     * @return boolean of whether or not the motor is within the tolerance range of the target rps
     */
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

    /**
     * Runs the launcher at a low "idle" speed so the wheels stay spun up while waiting to shoot.
     *
     * <p>This reduces time-to-speed for the next shot compared to starting from a full stop.
     * The idle setpoint is {@link frc.robot.Konstants.LauncherConstants#kIdleLauncherRPS}.
     */
    public void idleLauncher() {

        targetMotorRPS = kIdleLauncherRPS;
        launcherVelocityControl.Velocity = targetMotorRPS;
        launchermotor.setControl(launcherVelocityControl);
        launcherMotorStatus = "Idled";
    }

    //Sends subsystem to the Smart Dashboard
    @Override
    public void periodic() {
        SmartDashboard.putData("Static Launcher", this);
    }

    //Sends data to the Smart Dashboard
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty(
            "Launcher Status", 
            () -> launcherMotorStatus,
            null);
        builder.addBooleanProperty(
            "Is at launcher speed",
            () -> isLauncherAtSpeed(),
            null);
        builder.addDoubleProperty(
            "Launcher Velocity",
            () -> launchermotor.getVelocity().getValueAsDouble()*(2*Math.PI*kWheelRadius),
            null);
        builder.addDoubleProperty(
            "Target RPS",
            () -> targetMotorRPS,
            null);
    }
}
