package frc.robot.subsystems;

import static frc.robot.Konstants.LauncherConstants.Slot0;
import static frc.robot.Konstants.LauncherConstants.Slot1;
import static frc.robot.Konstants.LauncherConstants.kLauncherA;
import static frc.robot.Konstants.LauncherConstants.kLauncherV;
import static frc.robot.Konstants.LauncherConstants.kLauncherS;
import static frc.robot.Konstants.LauncherConstants.kWheelRadius;
import static frc.robot.Konstants.LauncherConstants.kShooterTolerance;
import static frc.robot.Konstants.LauncherConstants.kCoastLauncherRPS;
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
import static frc.robot.Ports.LauncherPorts.kFixedLauncherMotor;
import static frc.robot.Ports.LauncherPorts.kFixedLauncherMotorFollower;

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
