package frc.robot.subsystems.intake;

// Imports from robot
import static frc.robot.Konstants.IntakeConstants.kPositionerKG;
import static frc.robot.Konstants.IntakeConstants.kPositionerKp;
import static frc.robot.Konstants.IntakeConstants.kPositionerKi;
import static frc.robot.Konstants.IntakeConstants.kPositionerKd;
import static frc.robot.Konstants.IntakeConstants.kPositionerKa;
import static frc.robot.Konstants.IntakeConstants.kPositionerKs;
import static frc.robot.Konstants.IntakeConstants.kPositionerKv;
import static frc.robot.Konstants.IntakeConstants.kPositionerPeakForwardVoltage;
import static frc.robot.Konstants.IntakeConstants.kPositionerPeakReverseVoltage;
import static frc.robot.Konstants.IntakeConstants.kPositionerMMCruiseVelocity;
import static frc.robot.Konstants.IntakeConstants.kPositionerMMAcceleration;
import static frc.robot.Konstants.IntakeConstants.kPositionerMMJerk;
import static frc.robot.Konstants.IntakeConstants.kPositionerMMExpoKV;
import static frc.robot.Konstants.IntakeConstants.kPositionerMMExpoKA;
import static frc.robot.Konstants.IntakeConstants.kPositionerSupplyCurrentLimit;
import static frc.robot.Konstants.IntakeConstants.kPositionerStatorCurrentLimit;
import static frc.robot.Konstants.IntakeConstants.kPositionerEncoderDiscontinuityPoint;
import static frc.robot.Konstants.IntakeConstants.kPositionerEncoderGearRatio;
import static frc.robot.Konstants.IntakeConstants.kPositionerEncoderOffset;
import static frc.robot.Konstants.IntakeConstants.kPositionerEncoderInverted;
import static frc.robot.Konstants.IntakeConstants.kPositionerGainSchedulerErrorThreshold;
import static frc.robot.Konstants.IntakeConstants.kPositionerPositionTolerance;
import static frc.robot.Ports.pickupOBPorts.kPositionerEncoder;
import static frc.robot.Ports.pickupOBPorts.kPositionerMotor;
import static frc.robot.Ports.pickupOBPorts.kPositionerFollowerMotor;

import frc.lib.commands.PathPlannerCommands;
import frc.lib.subsystems.PathplannerSubsystem;
import frc.robot.Konstants.IntakeConstants.IntakePosition;

// Imports from Phoenix 6
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import org.littletonrobotics.junction.Logger;

// Imports from WPILib
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
	private final CANcoder positionerEncoder;

	// Control requests
	private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0.0);
	private final Follower followerControl;

	// Position tracking
	private double motorTargetPosition;
	private double currentPositionerPosition = 0.0;

	private final StatusSignal<Angle> positionerAngleStatusSignal;
	private final StatusSignal<AngularVelocity> positionerAngularVelocityStatusSignal;

	private IntakePosition targetPositionEnum = IntakePosition.ZERO;

	public SK26IntakePivot()
	{
		// ========== CANcoder Configuration ==========
		positionerEncoder = new CANcoder(kPositionerEncoder.ID);
		CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
		MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
		magnetConfig.MagnetOffset = kPositionerEncoderOffset;
		magnetConfig.SensorDirection = kPositionerEncoderInverted
			? SensorDirectionValue.Clockwise_Positive
			: SensorDirectionValue.CounterClockwise_Positive;
		encoderConfig.MagnetSensor = magnetConfig;
		encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = kPositionerEncoderDiscontinuityPoint;
		positionerEncoder.getConfigurator().apply(encoderConfig);

		// ========== Positioner Motor Configuration ==========
		positionerMotor = new TalonFX(kPositionerMotor.ID);
		TalonFXConfiguration positionerConfig = new TalonFXConfiguration();

		// Motor output
		MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
		outputConfigs.NeutralMode = NeutralModeValue.Brake;
		outputConfigs.Inverted = InvertedValue.Clockwise_Positive;
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
			.withGainSchedBehavior(GainSchedBehaviorValue.ZeroOutput);

		// Voltage limits
		positionerConfig.Voltage = new VoltageConfigs()
			.withPeakForwardVoltage(kPositionerPeakForwardVoltage)
			.withPeakReverseVoltage(kPositionerPeakReverseVoltage);

		// Motion Magic configuration
		positionerConfig.MotionMagic = new MotionMagicConfigs()
			.withMotionMagicCruiseVelocity(kPositionerMMCruiseVelocity)
			.withMotionMagicAcceleration(kPositionerMMAcceleration)
			.withMotionMagicJerk(kPositionerMMJerk)
			.withMotionMagicExpo_kV(kPositionerMMExpoKV)
			.withMotionMagicExpo_kA(kPositionerMMExpoKA);

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
			.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
			.withFeedbackRemoteSensorID(kPositionerEncoder.ID)
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
		targetPositionEnum = IntakePosition.ZERO;

		addPathPlannerCommands();
	}

	/**
	 * Sets the target position for the positioner motor using Motion Magic.
	 * @param targetPosition Target position in rotations
	 */
	public void setTargetPosition(double targetPosition)
	{
		motorTargetPosition = targetPosition;
		positionerMotor.setControl(motionMagicControl.withPosition(targetPosition));
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
		PathPlannerCommands.addCommand("Intake Stow", this.setIntakePivotTargetCommand(IntakePosition.ZERO).withName("IntakeStowAuton"));
		System.out.println("[SK26IntakePivot] PathPlanner commands added");
	}
}
