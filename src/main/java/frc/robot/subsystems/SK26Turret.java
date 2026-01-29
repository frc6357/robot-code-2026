package frc.robot.subsystems;

// Imports from the robot
import static frc.robot.Konstants.TurretConstants.*;
import static frc.robot.Ports.LauncherPorts.kTurretMotor;
import static frc.robot.Ports.LauncherPorts.kTurretEncoder;

// Imports from (the goat) Phoenix
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

public class SK26Turret extends SubsystemBase
{
    // Motor objects
    private final TalonFX turretMotor = new TalonFX(kTurretMotor.ID);
    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    // Absolute encoder (CANcoder) - the ONLY position feedback source
    private final CANcoder turretEncoder = new CANcoder(kTurretEncoder.ID);

    // WPILib PID controller (uses CANcoder feedback, not motor encoder)
    private final PIDController pidController = new PIDController(kTurretP, kTurretI, kTurretD);

    // Motor output control
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

    // Target angle (starts at 0.0)
    private double targetAngleDeg = 0.0;

    public SK26Turret()
    {
        // ========== CANcoder Configuration ==========
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        // Set the magnet offset so that 0 corresponds to turret "forward"
        magnetConfig.MagnetOffset = kTurretEncoderOffset;
        // Set sensor direction (adjust if encoder reads backwards)
        magnetConfig.SensorDirection = kTurretEncoderInverted 
            ? SensorDirectionValue.Clockwise_Positive 
            : SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor = magnetConfig;
        
        turretEncoder.getConfigurator().apply(encoderConfig);

        // ========== Motor Configuration ==========
        // Motor output configs (no internal PID - we use external PID with CANcoder)
        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
        outputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorConfig.MotorOutput = outputConfigs;

        // Applying configs
        turretMotor.getConfigurator().apply(motorConfig);

        // ========== PID Configuration ==========
        // Set tolerance for "at target" check
        pidController.setTolerance(kTurretAngleTolerance);
        // Enable continuous input for wrapping (-180 to 180)
        pidController.enableContinuousInput(-180.0, 180.0);

        // ========== Move to Zero on Boot ==========
        // Set target to zero position
        targetAngleDeg = 0.0;
        pidController.setSetpoint(targetAngleDeg);
    }

    // Move the turret to an absolute angle, in degrees
    public void setAngleDegrees(double angleDeg)
    {
        angleDeg = MathUtil.clamp(angleDeg, kTurretMinPosition, kTurretMaxPosition);
        targetAngleDeg = angleDeg;
        pidController.setSetpoint(targetAngleDeg);
    }

    // Stop turret motion
    public void stop()
    {
        turretMotor.stopMotor();
    }

    // Returns the current turret angle from the CANcoder (degrees)
    // This is now the ONLY source of position feedback
    public double getAngleDegrees()
    {
        return getAbsoluteAngleDegrees();
    }

    /**
     * Returns the absolute turret angle from the CANcoder (degrees).
     * Normalized to the range -180 to +180.
     * Accounts for the 2:1 encoder gear ratio.
     */
    public double getAbsoluteAngleDegrees()
    {
        // CANcoder returns position in rotations
        double encoderRotations = turretEncoder.getAbsolutePosition().getValueAsDouble();
        
        // Account for 2:1 gear ratio: 2 encoder rotations = 1 turret rotation
        // So turret rotations = encoder rotations / 2
        double turretRotations = encoderRotations / kEncoderGearRatio;
        
        // Convert to degrees (0 to 360)
        double degrees = turretRotations * 360.0;
        
        // Normalize to -180 to +180 range
        if (degrees > 180.0)
        {
            degrees -= 360.0;
        }
        
        return degrees;
    }

    // Returns the current target angle in degrees
    public double getTargetAngleDegrees()
    {
        return targetAngleDeg;
    }

    // Check to see if the turret is at its target position
    public boolean atTarget()
    {
        return pidController.atSetpoint();
    }

    @Override
    public void periodic()
    {
        // ========== Run PID Loop ==========
        // Calculate motor output based on CANcoder position feedback
        double currentAngle = getAbsoluteAngleDegrees();
        double output = pidController.calculate(currentAngle);
        
        // Clamp output to safe range
        output = MathUtil.clamp(output, -kMaxTurretOutput, kMaxTurretOutput);
        
        // Invert output if motor direction is reversed relative to encoder
        if (kTurretMotorInverted)
        {
            output = -output;
        }
        
        // Apply output to motor
        turretMotor.setControl(dutyCycleControl.withOutput(output));

        // ========== Dashboard ==========
        SmartDashboard.putNumber("Turret Angle (deg)", currentAngle);
        SmartDashboard.putNumber("Turret Target (deg)", targetAngleDeg);
        SmartDashboard.putBoolean("Turret At Target", atTarget());
        SmartDashboard.putNumber("Turret Output", output);
    }
}
