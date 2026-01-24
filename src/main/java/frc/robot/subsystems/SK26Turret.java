package frc.robot.subsystems;

// Imports from the robot
import static frc.robot.Konstants.TurretConstants.*;
import static frc.robot.Ports.LauncherPorts.kTurretMotor;
import static frc.robot.Ports.LauncherPorts.kTurretEncoder;

// Imports from (the goat) Phoenix
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

// Imports from WPILib
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SK26Turret extends SubsystemBase
{
    // Motor objects
    private final TalonFX turretMotor = new TalonFX(kTurretMotor.ID);
    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    // Absolute encoder (CANcoder) - reads turret position directly
    private final CANcoder turretEncoder = new CANcoder(kTurretEncoder.ID);

    // Target angle (starts at 0.0)
    private double targetAngleDeg = 0.0;

    public SK26Turret()
    {
        // ========== CANcoder Configuration ==========
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        // Set the magnet offset so that 0 corresponds to turret "forward"
        // TODO: Measure this value with the turret physically at the zero position
        magnetConfig.MagnetOffset = kTurretEncoderOffset;
        // Set sensor direction (adjust if encoder reads backwards)
        magnetConfig.SensorDirection = kTurretEncoderInverted 
            ? SensorDirectionValue.Clockwise_Positive 
            : SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor = magnetConfig;
        
        turretEncoder.getConfigurator().apply(encoderConfig);

        // ========== Motor Configuration ==========
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

        // ========== Sync Motor Encoder to Absolute Encoder ==========
        // Read the absolute turret position from CANcoder and set motor encoder to match
        syncMotorToAbsoluteEncoder();

        // ========== Move to Zero on Boot ==========
        // Command the turret to move to the zero position
        setAngleDegrees(0.0);
    }

    /**
     * Syncs the motor's internal encoder to the CANcoder's absolute position.
     * Call this at startup or if the motor encoder drifts.
     */
    public void syncMotorToAbsoluteEncoder()
    {
        // Get the normalized absolute turret angle (-180 to +180)
        double absoluteTurretDegrees = getAbsoluteAngleDegrees();
        
        // Set the motor encoder position to the equivalent motor rotations
        turretMotor.setPosition(degreesToMotorRotations(absoluteTurretDegrees));
        
        // Also update the target to current position
        targetAngleDeg = absoluteTurretDegrees;
    }

    /**
     * Returns the absolute turret angle from the CANcoder (degrees).
     * Normalized to the range -180 to +180.
     * This is the "ground truth" position.
     */
    public double getAbsoluteAngleDegrees()
    {
        // CANcoder returns position in rotations (0 to 1 range after offset)
        double rotations = turretEncoder.getAbsolutePosition().getValueAsDouble();
        
        // Convert to degrees (0 to 360)
        double degrees = rotations * 360.0;
        
        // Normalize to -180 to +180 range
        if (degrees > 180.0)
        {
            degrees -= 360.0;
        }
        
        return degrees;
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
        SmartDashboard.putNumber("Turret Absolute Angle (deg)", getAbsoluteAngleDegrees());
        SmartDashboard.putNumber("Turret Target (deg)", targetAngleDeg);
        SmartDashboard.putBoolean("Turret At Target", atTarget());
        SmartDashboard.putNumber("Turret Velocity (deg/s)", motorRotationsToDegrees(turretMotor.getVelocity().getValueAsDouble()));
    }
}
