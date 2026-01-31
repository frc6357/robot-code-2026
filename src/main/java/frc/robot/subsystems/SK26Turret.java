package frc.robot.subsystems;

// Imports from the robot
import static frc.robot.Konstants.TurretConstants.*;
import static frc.robot.Ports.LauncherPorts.kTurretMotor;
import static frc.robot.Ports.LauncherPorts.kTurretEncoder;

// Imports from Phoenix
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

// Imports from WPILib
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Turret subsystem using CANcoder absolute encoder as the ONLY position feedback.
 * Encoder has 2:1 gear ratio (2 encoder rotations = 1 turret rotation).
 * Turret range: -180 to +180 degrees.
 */
public class SK26Turret extends SubsystemBase
{
    // Motor
    private final TalonFX turretMotor = new TalonFX(kTurretMotor.ID);

    // Absolute encoder (CANcoder) - the ONLY position feedback source
    private final CANcoder turretEncoder = new CANcoder(kTurretEncoder.ID);

    // WPILib PID controller (runs in periodic, uses CANcoder feedback)
    private final PIDController pidController = new PIDController(kTurretP, kTurretI, kTurretD);

    // Motor output control
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

    // Target angle in degrees
    private double targetAngleDeg = 0.0;

    public SK26Turret()
    {
        // ========== CANcoder Configuration ==========
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        magnetConfig.MagnetOffset = kTurretEncoderOffset;
        magnetConfig.SensorDirection = kTurretEncoderInverted 
            ? SensorDirectionValue.Clockwise_Positive 
            : SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor = magnetConfig;
        
        turretEncoder.getConfigurator().apply(encoderConfig);

        // ========== Motor Configuration ==========
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
        outputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorConfig.MotorOutput = outputConfigs;
        turretMotor.getConfigurator().apply(motorConfig);

        // ========== PID Configuration ==========
        pidController.setTolerance(kTurretAngleTolerance);
        pidController.setIZone(8); // 5 degrees

        // ========== Initialize ==========
        // Set initial target to current position (don't move on boot)
        targetAngleDeg = getAngleDegrees();
        pidController.setSetpoint(targetAngleDeg);
    }

    /**
     * Returns the current turret angle from the CANcoder (degrees).
     * Accounts for 2:1 gear ratio (2 encoder rotations = 1 turret rotation).
     * Range: -180 to +180 degrees.
     */
    public double getAngleDegrees()
    {
        // Use continuous position to track across full range
        double encoderRotations = turretEncoder.getPosition().getValueAsDouble();
        
        // 2:1 ratio: turret degrees = encoder rotations * (360 / 2) = encoder rotations * 180
        double turretDegrees = encoderRotations * (360.0 / kEncoderGearRatio);
        
        return turretDegrees;
    }

    /**
     * Command the turret to move to a specific angle.
     * @param angleDeg Target angle in degrees (-180 to +180)
     */
    public void setAngleDegrees(double angleDeg)
    {
        angleDeg = MathUtil.clamp(angleDeg, kTurretMinPosition, kTurretMaxPosition);
        targetAngleDeg = angleDeg;
        pidController.setSetpoint(targetAngleDeg);
    }

    /**
     * Command the turret to move to a specific angle, wrapping around if it exceeds limits.
     * If angle goes past +170, it wraps to -170 (and vice versa).
     * @param angleDeg Target angle in degrees
     */
    public void setAngleDegreesWrapped(double angleDeg)
    {
        // Calculate the total range (from -170 to +170 = 340 degrees)
        double range = kTurretMaxPosition - kTurretMinPosition;
        
        // Wrap the angle if it exceeds limits
        while (angleDeg > kTurretMaxPosition)
        {
            angleDeg -= range;
        }
        while (angleDeg < kTurretMinPosition)
        {
            angleDeg += range;
        }
        
        targetAngleDeg = angleDeg;
        pidController.setSetpoint(targetAngleDeg);
    }

    /**
     * Returns the current target angle in degrees.
     */
    public double getTargetAngleDegrees()
    {
        return targetAngleDeg;
    }

    /**
     * Check if the turret is at its target position.
     */
    public boolean atTarget()
    {
        return pidController.atSetpoint();
    }

    /**
     * Stop the turret motor.
     */
    public void stop()
    {
        turretMotor.stopMotor();
    }

    @Override
    public void periodic()
    {
        // ========== Run PID Loop ==========
        double currentAngle = getAngleDegrees();
        double output = (atTarget() ? 0.0 : pidController.calculate(currentAngle));
        
        // Clamp output for safety
        output = MathUtil.clamp(output, -kMaxTurretOutput, kMaxTurretOutput);
        
        // Invert if needed
        if (kTurretMotorInverted)
        {
            output = -output;
        }
        
        // Apply to motor
        turretMotor.setControl(dutyCycleControl.withOutput(output));

        // ========== Dashboard ==========
        SmartDashboard.putNumber("Turret Angle (deg)", currentAngle);
        SmartDashboard.putNumber("Turret Target (deg)", targetAngleDeg);
        SmartDashboard.putNumber("Turret Error (deg)", targetAngleDeg - currentAngle);
        SmartDashboard.putNumber("Turret Output", output);
        SmartDashboard.putBoolean("Turret At Target", atTarget());
    }
}
