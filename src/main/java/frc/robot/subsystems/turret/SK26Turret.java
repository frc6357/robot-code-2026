package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
// Imports from the robot
import static frc.robot.Konstants.TurretConstants.kEncoderGearRatio;
import static frc.robot.Konstants.TurretConstants.kMaxTurretMMAcceleration;
import static frc.robot.Konstants.TurretConstants.kMaxTurretMMJerk;
import static frc.robot.Konstants.TurretConstants.kMaxTurretMMVelocity;
import static frc.robot.Konstants.TurretConstants.kMaxTurretOutputVolts;
import static frc.robot.Konstants.TurretConstants.kTurretA;
import static frc.robot.Konstants.TurretConstants.kTurretAngleTolerance;
import static frc.robot.Konstants.TurretConstants.kTurretD;
import static frc.robot.Konstants.TurretConstants.kTurretEncoderInverted;
import static frc.robot.Konstants.TurretConstants.kTurretEncoderOffset;
import static frc.robot.Konstants.TurretConstants.kTurretGainSchedulerDeadbandDegrees;
import static frc.robot.Konstants.TurretConstants.kTurretI;
import static frc.robot.Konstants.TurretConstants.kTurretMaxPosition;
import static frc.robot.Konstants.TurretConstants.kTurretMinPosition;
import static frc.robot.Konstants.TurretConstants.kTurretMotorInverted;
import static frc.robot.Konstants.TurretConstants.kTurretP;
import static frc.robot.Konstants.TurretConstants.kTurretS;
import static frc.robot.Konstants.TurretConstants.kTurretV;
import static frc.robot.Ports.TurretPorts.kTurretEncoder;
import static frc.robot.Ports.TurretPorts.kTurretMotor;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
// Imports from Phoenix
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

// Imports from WPILib
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.Konstants.TurretConstants.TurretPosition;
import lombok.Getter;

/**
 * Turret subsystem using CANcoder absolute encoder as the ONLY position feedback.
 * Encoder has 2:1 gear ratio (2 encoder rotations = 1 turret rotation).
 * Turret range: -180 to +180 degrees.
 */
public class SK26Turret extends SubsystemBase
{
    // Motor (protected for simulation subclass access)
    protected final TalonFX turretMotor = new TalonFX(kTurretMotor.ID);
    
    // Absolute encoder (CANcoder) - the ONLY position feedback source
    // Protected for simulation subclass access
    protected final CANcoder turretEncoder = new CANcoder(kTurretEncoder.ID);

    @Getter
    boolean joystickWrapping = false;
    
    // MotionMagic position control request (Phoenix6 firmware-side closed loop)
    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0.0);

    // Voltage-only control for SysId (no closed loop)
    private final VoltageOut voltageControl = new VoltageOut(0.0);

    // Target angle in degrees
    private double targetAngleDeg = 0.0;

    // Cached per-cycle value to avoid redundant CAN reads
    private double cachedAngleDeg = 0.0;
    private double cachedVelocityDegPerSec = 0.0;

    private final StatusSignal<Angle> turretAngleStatusSignal;
    private final StatusSignal<AngularVelocity> turretAngularVelocityStatusSignal;

    // Create the SysId routine
    SysIdRoutine sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
        Volts.of(1).div(Seconds.of(0.25)), Volts.of(5), Seconds.of(1.35), // Use default config
        (state) -> Logger.recordOutput("SysIDs/TurretSysIdTestState", state.toString())
    ),
    new SysIdRoutine.Mechanism(
        (voltage) -> turretMotor.setControl(voltageControl.withOutput(voltage).withEnableFOC(true)),
        null, // No log consumer, since data is recorded by AdvantageKit
        this
    )
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

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

        // Motor output
        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
        outputConfigs.NeutralMode = NeutralModeValue.Coast;
        outputConfigs.Inverted = kTurretMotorInverted 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput = outputConfigs;

        motorConfig.ClosedLoopGeneral = new ClosedLoopGeneralConfigs()
            .withGainSchedErrorThreshold(Degrees.of(kTurretGainSchedulerDeadbandDegrees));

        // Closed-loop PID (Slot 0) — runs on the motor controller firmware
        motorConfig.Slot0 = new Slot0Configs()
        .withKP(kTurretP)
        .withKI(kTurretI)
        .withKD(kTurretD)
        .withKS(kTurretS)
        .withKV(kTurretV)
        .withKA(kTurretA)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
        .withGainSchedBehavior(GainSchedBehaviorValue.ZeroOutput);
        
        motorConfig.Voltage = new VoltageConfigs()
        .withPeakForwardVoltage(kMaxTurretOutputVolts)
        .withPeakReverseVoltage(-kMaxTurretOutputVolts);

        // FusedCANcoder feedback — motor firmware fuses rotor + CANcoder position
        motorConfig.Feedback = new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        .withFeedbackRemoteSensorID(kTurretEncoder.ID)
        .withSensorToMechanismRatio(kEncoderGearRatio)
        .withVelocityFilterTimeConstant(0.1);     // 2 encoder rotations = 1 turret rotation
        // .withRotorToSensorRatio(kTurretMotorGearRatio / kEncoderGearRatio); // motor → encoder ratio
        
        // Motion Magic velocity / acceleration (converted from deg/s → turret rot/s)
        motorConfig.MotionMagic = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(
            kMaxTurretMMVelocity.in(RotationsPerSecond))
        .withMotionMagicAcceleration(
            kMaxTurretMMAcceleration.in(RotationsPerSecondPerSecond))
        .withMotionMagicJerk(kMaxTurretMMJerk);
        
        turretMotor.getConfigurator().apply(motorConfig);
        
        turretAngleStatusSignal = turretMotor.getPosition();
        turretAngularVelocityStatusSignal = turretMotor.getVelocity();
        
        // ========== Initialize ==========
        // Set initial target to current position (don't move on boot)
        targetAngleDeg = getAngleDegrees();
        }
            
    @Override
    public void periodic()
    {
        // ========== Batch refresh all status signals in a single CAN frame ==========
        BaseStatusSignal.refreshAll(turretAngleStatusSignal, turretAngularVelocityStatusSignal);
        
        // ========== Cache sensor reads from refreshed signals ==========
        cachedAngleDeg = turretAngleStatusSignal.getValue().in(Degrees);
        cachedVelocityDegPerSec = turretAngularVelocityStatusSignal.getValue().in(DegreesPerSecond);

        // ========== Send MotionMagic target to motor firmware ==========
        // Convert target from degrees to turret rotations (mechanism units)
        double targetRotations = targetAngleDeg / 360.0;
        turretMotor.setControl(motionMagicControl.withPosition(targetRotations));

        telemeterize();
    }
    
    /**
     * Returns the current turret angle from the CANcoder (degrees).
     * Accounts for 2:1 gear ratio (2 encoder rotations = 1 turret rotation).
     * Range: -180 to +180 degrees.
     * Note: This forces a refresh - prefer using getCachedAngleDegrees() when possible.
     */
    private double getAngleDegrees()
    {
        // Use continuous position to track across full range
        double encoderDegrees = turretAngleStatusSignal.refresh().getValue().in(Degrees);
        
        // // 2:1 ratio: turret degrees = encoder rotations * (360 / 2) = encoder rotations * 180
        // double turretDegrees = encoderRotations * (360.0 / kEncoderGearRatio);
        
        return encoderDegrees;
    }

    /**
     * Returns the cached turret angle in degrees instead of reading from the CANcoder again.
     * @return Cached turret angle in degrees
     */
    public double getCachedAngleDegrees() {
        return cachedAngleDeg;
    }

    /**
     * Returns the cached angular velocity in degrees per second.
     * @return Cached angular velocity in degrees/second
     */
    public double getCachedAngularVelocityDegreesPerSecond() {
        return cachedVelocityDegPerSec;
    }

    /**
     * Returns the angular velocity in degrees per second.
     * Note: This forces a refresh - prefer using getCachedAngularVelocityDegreesPerSecond() when possible.
     */
    public double getAngularVelocityDegreesPerSecond() {
        double encoderVelocityDegpersec = turretAngularVelocityStatusSignal.refresh().getValue().in(DegreesPerSecond);
        return encoderVelocityDegpersec;
    }
    
    public void setAngleDegrees(TurretPosition angle) 
    {
        setAngleDegrees(angle.angle);
    }
    
    /**
     * Command the turret to move to a specific angle.
     * @param angleDeg Target angle in degrees (-180 to +180)
     */
    public void setAngleDegrees(double angleDeg)
    {
        angleDeg = MathUtil.clamp(angleDeg, kTurretMinPosition, kTurretMaxPosition);
        targetAngleDeg = angleDeg;
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
            joystickWrapping = true;
            angleDeg -= range;
        }
        while (angleDeg < kTurretMinPosition)
        {
            joystickWrapping = true;
            angleDeg += range;
        }
        
        targetAngleDeg = angleDeg;
    }
    

    /**
     * Returns the current target angle in degrees.
     */
    public double getTargetAngleDegrees()
    {
        return targetAngleDeg;
    }

    public double getTurretError() {
        return targetAngleDeg - cachedAngleDeg;
    }

    public double getMotorDutyCycle() {
        return turretMotor.getDutyCycle().getValueAsDouble();
    }

    public double getMotorVoltage() {
        return turretMotor.getMotorVoltage().getValueAsDouble();
    }

    /**
     * Check if the turret is at its target position.
     */
    public boolean atTarget()
    {
        return Math.abs(getTurretError()) <= kTurretAngleTolerance;
    }

    public boolean closeToTarget() {
        return Math.abs(getTurretError()) <= kTurretAngleTolerance * 2.0;
    }

    /**
     * Reset the controller and sync target to current position.
     * Useful after simulation initialization or when recovering from errors.
     */
    protected void resetController()
    {
        targetAngleDeg = getAngleDegrees();
    }

    /**
     * Stop the turret motor.
     */
    public void stop()
    {
        turretMotor.stopMotor();
    }


    public void telemeterize() {
        // Sends the needed Pose3d for the turret CAD model to correctly rotate the turret in the 3D visualization on the dashboard
        double angle = cachedAngleDeg;
        if(!Robot.isReal()) {
            Rotation3d turretRotation = new Rotation3d(0, 0, Math.toRadians(angle));
            Logger.recordOutput("Mechanisms/TurretSpinPose", new Pose3d(
                Translation3d.kZero.rotateAround(
                    new Translation3d(Inches.of(-7.05), Inches.of(7.05), Inches.of(0)), 
                    turretRotation),
                turretRotation));
        }
        
        Logger.recordOutput("Turret/Angle (deg)", cachedAngleDeg);
        Logger.recordOutput("Turret/Velocity (degpersec)", cachedVelocityDegPerSec);
        Logger.recordOutput("Turret/Target (deg)", getTargetAngleDegrees());
        Logger.recordOutput("Turret/At Target", atTarget());
        Logger.recordOutput("Turret/Error (deg)", getTurretError());
        Logger.recordOutput("Turret/ErrorAbs (deg)", Math.abs(getTurretError()));
        Logger.recordOutput("Turret/Motor Voltage Output", getMotorVoltage());
        Logger.recordOutput("Turret/Wrapping", isJoystickWrapping());
    }
}