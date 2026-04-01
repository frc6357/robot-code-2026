package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Konstants.IntakeConstants.kPositionerEncoderDiscontinuityPoint;
import static frc.robot.Konstants.IntakeConstants.kPositionerEncoderGearRatio;
import static frc.robot.Konstants.IntakeConstants.kPositionerEncoderInverted;
import static frc.robot.Konstants.IntakeConstants.kPositionerEncoderOffset;
import static frc.robot.Konstants.IntakeConstants.kPositionerGainSchedulerErrorThreshold;
// Imports from robot
import static frc.robot.Konstants.IntakeConstants.kPositionerKG;
import static frc.robot.Konstants.IntakeConstants.kPositionerKa;
import static frc.robot.Konstants.IntakeConstants.kPositionerKd;
import static frc.robot.Konstants.IntakeConstants.kPositionerKi;
import static frc.robot.Konstants.IntakeConstants.kPositionerKp;
import static frc.robot.Konstants.IntakeConstants.kPositionerKs;
import static frc.robot.Konstants.IntakeConstants.kPositionerKv;
import static frc.robot.Konstants.IntakeConstants.kPositionerPeakForwardVoltage;
import static frc.robot.Konstants.IntakeConstants.kPositionerPeakReverseVoltage;
import static frc.robot.Konstants.IntakeConstants.kPositionerPositionTolerance;
import static frc.robot.Konstants.IntakeConstants.kPositionerStatorCurrentLimit;
import static frc.robot.Konstants.IntakeConstants.kPositionerSupplyCurrentLimit;
import static frc.robot.Ports.pickupOBPorts.kPositionerEncoder;
import static frc.robot.Ports.pickupOBPorts.kPositionerFollowerMotor;
import static frc.robot.Ports.pickupOBPorts.kPositionerMotor;

import org.littletonrobotics.junction.Logger;

// Imports from Phoenix 6
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

// Imports from WPILib
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.commands.PathPlannerCommands;
import frc.lib.subsystems.PathplannerSubsystem;
import frc.robot.Konstants.IntakeConstants.IntakePosition;

/**
 * Subsystem for the intake pivot mechanism. Controls the positioner motor (and follower)
 * that raises and lowers the intake using Motion Magic position control.
 * Uses TalonFX (Kraken X44) motors.
 */
public class SK26IntakePivot extends SubsystemBase implements PathplannerSubsystem
{
	// Motors
	private final TalonFX positionerMotor;
	private final TalonFX positionerFollowerMotor;

	// Absolute encoder (CANcoder) — the sole position feedback source
	// private final CANcoder positionerEncoder;

	// Control requests
	private final PositionVoltage positionVoltageControl = new PositionVoltage(Rotations.of(0.0));
	private final Follower followerControl;

	// Position tracking
	private double motorTargetPosition;
	private double currentPositionerPosition = 0.0;

	private final StatusSignal<Angle> positionerAngleStatusSignal;
	private final StatusSignal<AngularVelocity> positionerAngularVelocityStatusSignal;

	private IntakePosition targetPositionEnum = IntakePosition.STOW;

	// ===== Stall Detection (ground hard-stop homing) =====
	// When targeting GROUND, if the motor stalls (velocity near zero) for STALL_TIME_SECONDS,
	// we know we've hit the physical hard-stop and the rotor sensor has drifted.
	// Reset the motor position to the known GROUND value so the PID stops pushing.
	private static final double STALL_VELOCITY_THRESHOLD_RPS = 0.5;
	private static final double STALL_TIME_SECONDS = 2.0;
	private static final double STALL_RESET_POSITION = IntakePosition.GROUND.rotations; // -0.235
	private final Timer stallTimer = new Timer();
	private boolean stallTimerRunning = false;
	private boolean stallResetApplied = false;

	public SK26IntakePivot()
	{
		// ========== CANcoder Configuration ==========
		// positionerEncoder = new CANcoder(kPositionerEncoder.ID);
		// CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
		// MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
		// magnetConfig.MagnetOffset = kPositionerEncoderOffset;
		// magnetConfig.SensorDirection = kPositionerEncoderInverted
		// 	? SensorDirectionValue.Clockwise_Positive
		// 	: SensorDirectionValue.CounterClockwise_Positive;
		// encoderConfig.MagnetSensor = magnetConfig;
		// encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = kPositionerEncoderDiscontinuityPoint;
		// positionerEncoder.getConfigurator().apply(encoderConfig);

		// ========== Positioner Motor Configuration ==========
		positionerMotor = new TalonFX(kPositionerMotor.ID);
		TalonFXConfiguration positionerConfig = new TalonFXConfiguration();

		// Motor output
		MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
		outputConfigs.NeutralMode = NeutralModeValue.Brake;
		outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
		positionerConfig.MotorOutput = outputConfigs;

		// PID configuration for position control (Motion Magic)
		positionerConfig.Slot0 = new Slot0Configs()
			.withKP(kPositionerKp)
			.withKI(kPositionerKi)
			.withKD(kPositionerKd)
			.withKS(kPositionerKs)
			.withKV(kPositionerKv)
			.withKA(kPositionerKa)
			.withKG(kPositionerKG)
			.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
			.withGravityType(GravityTypeValue.Arm_Cosine)
			.withGainSchedBehavior(GainSchedBehaviorValue.ZeroOutput);

		// Voltage limits
		positionerConfig.Voltage = new VoltageConfigs()
			.withPeakForwardVoltage(kPositionerPeakForwardVoltage)
			.withPeakReverseVoltage(kPositionerPeakReverseVoltage); // Phoenix says 16?

		// Motion Magic configuration
		// positionerConfig.MotionMagic = new MotionMagicConfigs()
		// 	.withMotionMagicCruiseVelocity(kPositionerMMCruiseVelocity)
		// 	.withMotionMagicAcceleration(kPositionerMMAcceleration)
		// 	.withMotionMagicJerk(kPositionerMMJerk)
		// 	.withMotionMagicExpo_kV(kPositionerMMExpoKV)
		// 	.withMotionMagicExpo_kA(kPositionerMMExpoKA);

		// Current limits
		positionerConfig.CurrentLimits = new CurrentLimitsConfigs()
			.withSupplyCurrentLimit(kPositionerSupplyCurrentLimit)
			.withSupplyCurrentLimitEnable(true)
			.withStatorCurrentLimit(kPositionerStatorCurrentLimit)
			.withStatorCurrentLimitEnable(true);

		// Feedback/closed-loop
		positionerConfig.ClosedLoopGeneral = new ClosedLoopGeneralConfigs()
			.withGainSchedErrorThreshold(kPositionerGainSchedulerErrorThreshold);

		positionerConfig.Feedback = new FeedbackConfigs()
			.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
			// .withFeedbackRemoteSensorID(kPositionerEncoder.ID)
			.withSensorToMechanismRatio(kPositionerEncoderGearRatio);

		positionerMotor.getConfigurator().apply(positionerConfig);

		// ========== Positioner Follower Motor Configuration ==========
		positionerFollowerMotor = new TalonFX(kPositionerFollowerMotor.ID);
		positionerFollowerMotor.getConfigurator().apply(
			positionerConfig
			.withMotorOutput(
				outputConfigs.withInverted(InvertedValue.Clockwise_Positive))
		);
		followerControl = new Follower(positionerMotor.getDeviceID(), MotorAlignmentValue.Opposed);
		positionerFollowerMotor.setControl(followerControl);

		// Initialize status signals from the motor
		positionerAngleStatusSignal = positionerMotor.getPosition();
		positionerAngularVelocityStatusSignal = positionerMotor.getVelocity();

		// Initialize position tracking
		motorTargetPosition = positionerAngleStatusSignal.refresh().getValue().in(edu.wpi.first.units.Units.Rotations);
		targetPositionEnum = IntakePosition.STOW;

		// 
		positionerMotor.setPosition(0.0);

		addPathPlannerCommands();
	}

	/**
	 * Sets the target position for the positioner motor using Motion Magic.
	 * @param targetPosition Target position in rotations
	 */
	public void setTargetPosition(double targetPosition)
	{
		motorTargetPosition = targetPosition;
		positionerMotor.setControl(positionVoltageControl.withPosition(targetPosition));
	}

	/**
	 * Returns the current position of the positioner motor from the encoder.
	 * @return Current position in rotations
	 */
	public double getCurrentPosition()
	{
		return currentPositionerPosition;
	}

	/**
	 * Returns the current target position for the positioner motor.
	 * @return Target position in rotations
	 */
	public double getTargetPosition()
	{
		return motorTargetPosition;
	}

	/**
	 * Returns true if the positioner motor is within tolerance of the target position.
	 */
	public boolean isPositionerAtTarget()
	{
		return Math.abs(getTargetPosition() - getCurrentPosition()) < kPositionerPositionTolerance;
	}

	/**
	 * Sets the positioner to a preset intake position.
	 */
	public void setPositionerPosition(IntakePosition angle)
	{
		targetPositionEnum = angle;
		setTargetPosition(angle.rotations);
	}

	/**
	 * Command factory to set the pivot to a preset position.
	 */
	public Command setIntakePivotTargetCommand(IntakePosition angle)
	{
		return this.runOnce(() -> setPositionerPosition(angle));
	}

	/**
	 * Returns the current target position enum.
	 */
	public IntakePosition getPositionerTargetEnum()
	{
		return targetPositionEnum;
	}

	/**
	 * Stops the positioner motor.
	 */
	public void stopPositioner()
	{
		positionerMotor.stopMotor();
	}

	@Override
	public void periodic()
	{
		// Cache position reading
		currentPositionerPosition = positionerAngleStatusSignal.refresh().getValue().in(edu.wpi.first.units.Units.Rotations);

		// ===== Stall detection for ground hard-stop homing =====
		// Only active when we're targeting GROUND and haven't already reset this cycle.
		// The motor's rotor sensor drifts, so the reported position undershoots the real
		// GROUND position. The PID keeps pushing into the hard-stop, stalling the motor.
		// After 2 seconds of stall we know we're physically at ground, so we tell the
		// motor "you are at -0.235" and the error drops to zero.
		double velocityRPS = Math.abs(positionerAngularVelocityStatusSignal.refresh().getValue().in(RotationsPerSecond));
		boolean targetingGround = (targetPositionEnum == IntakePosition.GROUND);
		boolean motorStalled = velocityRPS < STALL_VELOCITY_THRESHOLD_RPS;
		boolean stillHasError = !isPositionerAtTarget(); // PID is actively trying to move

		if (targetingGround && motorStalled && stillHasError && !stallResetApplied) {
			// Start / continue the stall timer
			if (!stallTimerRunning) {
				stallTimer.restart();
				stallTimerRunning = true;
			}

			if (stallTimer.hasElapsed(STALL_TIME_SECONDS)) {
				// We've been stalled long enough — reset position to known ground value
				positionerMotor.setPosition(STALL_RESET_POSITION);
				motorTargetPosition = STALL_RESET_POSITION;
				stallResetApplied = true;
				Logger.recordOutput("Intake/StallResetTriggered", true);
			}
		} else if (!targetingGround || !motorStalled || !stillHasError) {
			// Motor is moving or we're no longer targeting ground — reset stall tracking
			stallTimer.stop();
			stallTimerRunning = false;
			stallResetApplied = false;
		}

		Logger.recordOutput("Intake/StallResetApplied", stallResetApplied);
		logOutputs();
	}

	private void logOutputs()
	{
		Logger.recordOutput("Intake/Positioner Position (rot)", getCurrentPosition());
		Logger.recordOutput("Intake/Positioner Target Position (rot)", getTargetPosition());
		Logger.recordOutput("Intake/Positioner At Target", isPositionerAtTarget());
		Logger.recordOutput("Intake/Positioner Velocity (RPS)", positionerAngularVelocityStatusSignal.refresh().getValue().in(RotationsPerSecond));
		Logger.recordOutput("Intake/Positioner Error (rot)", getTargetPosition() - getCurrentPosition());
		Logger.recordOutput("Intake/Intake Target (enum)", getPositionerTargetEnum());
	}

	@Override
	public void addPathPlannerCommands()
	{
		PathPlannerCommands.addCommand("Intake Deploy", this.setIntakePivotTargetCommand(IntakePosition.GROUND).withName("IntakeDeployAuton"));
		PathPlannerCommands.addCommand("Intake Stow", this.setIntakePivotTargetCommand(IntakePosition.STOW).withName("IntakeStowAuton"));
		System.out.println("[SK26IntakePivot] PathPlanner commands added");
	}
}
