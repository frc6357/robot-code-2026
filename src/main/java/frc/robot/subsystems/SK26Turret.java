package frc.robot.subsystems;

import static frc.robot.Ports.LauncherPorts.kTurretMotor;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SK26Turret extends SubsystemBase 
{
    //TODO: Moved motor variable declaration outside of constructor since it will be needed in other methods
    //TODO: Changed motor type to TalonFX since we're using krakens

    // Motor 
    private TalonFX turretMotor;
    // Some physical constants
    private final MotionMagicDutyCycle motionMagic = new MotionMagicDutyCycle(0);
    private static final double kMotorRotPerTurretRot = 12.8; //Ideally motor rotations per turret rotation
    private static final double kDegreesPerMotorRotation = 360.0 / kMotorRotPerTurretRot; 
    private static final double kMinAngleDegrees = -170.0; //TODO Replace my generic angle values with true value
    private static final double kMaxAngleDegrees = 170.0;
    private static final double kCruiseVelocity = 60; //rotations/sec
    private static final double kAcceleration = 30000; //rotations/sec^2
    // PID values
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.0;

    public SK26Turret() 
    {
        turretMotor = new TalonFX(kTurretMotor.ID);

        configureMotor();
        zeroTurret();
    }

    private void configureMotor() 
    {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kF;

        config.MotionMagic.MotionMagicCruiseVelocity = kCruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = kAcceleration;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degreesToMotorRotations(kMaxAngleDegrees);
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degreesToMotorRotations(kMinAngleDegrees);

        turretMotor.getConfigurator().apply(config);
        turretMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setAngleDegrees(double angleDegrees)
    {
        angleDegrees = clamp(angleDegrees, kMinAngleDegrees, kMaxAngleDegrees);
        motionMagic.Position = degreesToMotorRotations(angleDegrees);
        turretMotor.setControl(motionMagic);
    }

    public double getAngleDegrees()
    {
        return motorRotationsToDegrees(turretMotor.getPosition().getValueAsDouble());
    }

    private double getClosedLoopErrorDegrees()
    {
        return motorRotationsToDegrees(turretMotor.getClosedLoopError().getValue());
    }

    private static double degreesToMotorRotations(double degrees)
    {
        return degrees / kDegreesPerMotorRotation;
    }

    private static double motorRotationsToDegrees(double rotations)
    {
        return rotations * kDegreesPerMotorRotation;
    }

    private static double clamp(double val, double min, double max)
    {
        return Math.max(min, Math.min(max, val));
    }

    public void zeroTurret()
    {
        turretMotor.setPosition(0);
    }

    public void manualRotate(double dutyCycle)
    {
        turretMotor.set(dutyCycle);
    }

    public void initialize() {
        
    }

    @Override
    public void periodic() {

    }

    public void runLauncher(double speed) {
        
    }

    public void stopLauncher() {

    }

}
