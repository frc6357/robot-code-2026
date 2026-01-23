package frc.robot.subsystems;

// Imports from the robot
import static frc.robot.Konstants.TurretConstants.*;
import static frc.robot.Ports.LauncherPorts.kTurretMotor;

// Imports from (the goat) Phoenix
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

// Imports from WPILib
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SK26Turret extends SubsystemBase
{
    // Motor objects
    private final TalonFX turretMotor = new TalonFX(kTurretMotor.ID);
    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    // Target angle (starts at 0.0)
    private double targetAngleDeg = 0.0;

    public SK26Turret()
    {
        // Motor output configs
        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
        outputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorConfig.MotorOutput = outputConfigs;

        //PID configs
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = kTurretP;
        slot0.kI = kTurretI;
        slot0.kD = kTurretD;
        motorConfig.Slot0 = slot0;

        // Soft limit configurations (no head snapping ideally)
        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();

        softLimits.ForwardSoftLimitEnable = true;
        softLimits.ReverseSoftLimitEnable = true;

        softLimits.ForwardSoftLimitThreshold = degreesToMotorRotations(kTurretMaxPosition);

        softLimits.ReverseSoftLimitThreshold = degreesToMotorRotations(kTurretMinPosition);
        motorConfig.SoftwareLimitSwitch = softLimits;

        // Applying configs
        turretMotor.getConfigurator().apply(motorConfig);

        // Zero on boot (adjust for absolute encoder)
        turretMotor.setPosition(0.0);
    }

    // Move the turret to an absolute angle, in degrees
    public void setAngleDegrees(double angleDeg)
    {
        angleDeg = MathUtil.clamp(angleDeg, kTurretMinPosition, kTurretMaxPosition);
        targetAngleDeg = angleDeg;

        turretMotor.setControl(new PositionDutyCycle(degreesToMotorRotations(angleDeg)));
    }

    // Stop turret motion (if needed, might delete later)
    public void stop()
    {
        turretMotor.stopMotor();
    }

    // Just returns the current angle degrees
    public double getAngleDegrees()
    {
        return motorRotationsToDegrees(turretMotor.getPosition().getValueAsDouble());
    }

    // Returns the current target angle in degrees
    public double getTargetAngleDegrees()
    {
        return targetAngleDeg;
    }

    // Check to see if the turret is actually where its target is
    public boolean atTarget()
    {
        return Math.abs(getAngleDegrees() - targetAngleDeg) <= kTurretAngleTolerance;
    }

    // Unit conversion
    private double degreesToMotorRotations(double degrees)
    {
        return (degrees / 360.0) * kGearRatio;
    }
    private double motorRotationsToDegrees(double rotations)
    {
        return (rotations / kGearRatio) * 360.0;
    }


    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Turret Angle (deg)", getAngleDegrees());
        SmartDashboard.putNumber("Turret Target (deg)", targetAngleDeg);
        SmartDashboard.putBoolean("Turret At Target", atTarget());
        SmartDashboard.putNumber("Turret Velocity (deg/s)", motorRotationsToDegrees(turretMotor.getVelocity().getValueAsDouble()));
    }
}
