package frc.robot.subsystems.intake;

// Imports from robot
import static frc.robot.Konstants.IntakeConstants.kMaxIntakeVoltage;
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
import static frc.robot.Konstants.IntakeConstants.IntakePosition.GROUND;
import static frc.robot.Konstants.IntakeConstants.kPositionerStatorCurrentLimit;
import static frc.robot.Konstants.IntakeConstants.kPositionerSensorToMechanismRatio;
import static frc.robot.Konstants.IntakeConstants.kPositionerGainSchedulerErrorThreshold;
import static frc.robot.Konstants.IntakeConstants.kPositionerPositionTolerance;
import static frc.robot.Konstants.IntakeConstants.kIntakeSupplyCurrentLimit;
import static frc.robot.Konstants.IntakeConstants.kIntakeStatorCurrentLimit;
import static frc.robot.Ports.pickupOBPorts.kIntakeMotor;
import static frc.robot.Ports.pickupOBPorts.kPositionerMotor;
import static frc.robot.Ports.pickupOBPorts.kPositionerFollowerMotor;

import frc.lib.commands.PathPlannerCommands;
import frc.lib.subsystems.PathplannerSubsystem;
import frc.robot.Konstants.IntakeConstants.IntakePosition;

// Imports from Phoenix 6
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import org.littletonrobotics.junction.Logger;

// Imports from WPILib
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.RotationsPerSecond;

/**
 * Subsystem for the pickup mechanism of the robot, which includes a positioner motor and an intake motor.
 * Uses Motion Magic control with relative encoder feedback for smooth position tracking.
 * The positioner motor is responsible for moving the pickup mechanism to the correct position,
 * while the intake motor is responsible for intaking game pieces.
 * Uses TalonFX (Kraken X44) motors.
 */
public class SK26Intake extends SubsystemBase implements PathplannerSubsystem 
{
	// Motors
	private final TalonFX positionerMotor;
	private final TalonFX positionerFollowerMotor;
	private final TalonFX intakeMotor;

	// Control requests
	private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0.0);
	private final VoltageOut intakeVoltageControl = new VoltageOut(0);
	private final Follower followerControl;

	// Position tracking
	private double motorTargetPosition;
	private double currentPositionerPosition = 0.0;

	// Target voltage for the intake motor
	private double targetVoltage = 0.0;

	private final StatusSignal<Angle> positionerAngleStatusSignal;
	private final StatusSignal<AngularVelocity> positionerAngularVelocityStatusSignal;

	private String targetPositionEnum = "Off";

	// // Preferences for tuning
	// final Pref<Double> positionerKp = SKPreferences.attach("Intake/positionerKp", 0.0)
	// 	.onChange((newValue) -> reconfigurePositioner());
	// final Pref<Double> positionerKi = SKPreferences.attach("Intake/positionerKi", 0.0)
	// 	.onChange((newValue) -> reconfigurePositioner());
	// final Pref<Double> positionerKd = SKPreferences.attach("Intake/positionerKd", 0.0)
	// 	.onChange((newValue) -> reconfigurePositioner());
	// final Pref<Double> positionerKs = SKPreferences.attach("Intake/positionerKs", 0.0)
	// 	.onChange((newValue) -> reconfigurePositioner());
	// final Pref<Double> positionerKv = SKPreferences.attach("Intake/positionerKv", 0.0)
	// 	.onChange((newValue) -> reconfigurePositioner());
	// final Pref<Double> positionerKa = SKPreferences.attach("Intake/positionerKa", 0.0)
	// 	.onChange((newValue) -> reconfigurePositioner());
	// final Pref<Double> maxVelocity = SKPreferences.attach("Intake/maxVelocity", 2.0)
	// 	.onChange((newValue) -> reconfigurePositioner());
	// final Pref<Double> maxAcceleration = SKPreferences.attach("Intake/maxAcceleration", 4.0)
	// 	.onChange((newValue) -> reconfigurePositioner());
	// final Pref<Double> positionTolerance = SKPreferences.attach("Intake/positionTolerance", 0.1);

	// private void reconfigurePositioner() 
	// {
	// 	Slot0Configs slot0 = new Slot0Configs();
	// 	slot0.kP = positionerKp.get();
	// 	slot0.kI = positionerKi.get();
	// 	slot0.kD = positionerKd.get();
	// 	slot0.kS = positionerKs.get();
	// 	slot0.kV = positionerKv.get();
	// 	slot0.kA = positionerKa.get();
	// 	slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
	// 	slot0.GainSchedBehavior = GainSchedBehaviorValue.ZeroOutput;
	// 	positionerMotor.getConfigurator().apply(slot0);

	// 	MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
	// 	mmConfigs.withMotionMagicCruiseVelocity(maxVelocity.get());
	// 	mmConfigs.withMotionMagicAcceleration(maxAcceleration.get());
	// 	positionerMotor.getConfigurator().apply(mmConfigs);
	// }

	public SK26Intake() 
	{
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
			.withSensorToMechanismRatio(kPositionerSensorToMechanismRatio);

		positionerMotor.getConfigurator().apply(positionerConfig);
		positionerMotor.setPosition(IntakePosition.ZERO.rotations);

		// ========== Positioner Follower Motor Configuration ==========
		positionerFollowerMotor = new TalonFX(kPositionerFollowerMotor.ID);		
		positionerFollowerMotor.getConfigurator().apply(
			positionerConfig
			.withMotorOutput(
				outputConfigs.withInverted(InvertedValue.Clockwise_Positive))
		);
		followerControl = new Follower(positionerMotor.getDeviceID(), MotorAlignmentValue.Opposed);
		positionerFollowerMotor.setControl(followerControl);

		// ========== Intake Roller Motor Configuration ==========
		intakeMotor = new TalonFX(kIntakeMotor.ID);
		TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
		intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		intakeConfig.CurrentLimits = new CurrentLimitsConfigs()
			.withSupplyCurrentLimit(kIntakeSupplyCurrentLimit)
			.withSupplyCurrentLimitEnable(true)
			.withStatorCurrentLimit(kIntakeStatorCurrentLimit)
			.withStatorCurrentLimitEnable(true);
		intakeMotor.getConfigurator().apply(intakeConfig);

		// Initialize status signals for efficient polling
		positionerAngleStatusSignal = positionerMotor.getPosition();
		positionerAngularVelocityStatusSignal = positionerMotor.getVelocity();

		// Initialize position tracking
		motorTargetPosition = IntakePosition.ZERO.rotations;
		targetPositionEnum = IntakePosition.ZERO.name();

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
		targetPositionEnum = angle.name();
		setTargetPosition(angle.rotations);
	}

	public Command setIntakePivotTargetCommand(IntakePosition angle) 
	{
		return this.runOnce(() -> setPositionerPosition(angle));
	}

	/**
	 * Sets the intake motor voltage directly.
	 * @param voltage Voltage to apply (clamped to max intake voltage)
	 */
	public void setIntakeVoltage(double voltage) 
	{
		voltage = MathUtil.clamp(voltage, -kMaxIntakeVoltage, kMaxIntakeVoltage);
		targetVoltage = voltage;
		intakeMotor.setControl(intakeVoltageControl.withOutput(voltage));
	}

	public Command runAtVoltageCommand(double voltage) 
	{
		return this.runEnd(() -> setIntakeVoltage(voltage), () -> stopIntake());
	}

	/**
	 * Stops the intake motor.
	 */
	public void stopIntake()
	{
		setIntakeVoltage(0.0);
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
		Logger.recordOutput("Intake/Intake Voltage", targetVoltage);
		Logger.recordOutput("Intake/Intake Velocity (RPS)", intakeMotor.getVelocity().getValueAsDouble());
		Logger.recordOutput("Intake/Intake Target (enum)", targetPositionEnum);
	}

	@Override
	public void addPathPlannerCommands() 
	{
		PathPlannerCommands.addCommand("Intake Deploy", this.setIntakePivotTargetCommand(GROUND));
		PathPlannerCommands.addCommand("Intake Stow", this.setIntakePivotTargetCommand(IntakePosition.ZERO));
		System.out.println("[SK26Intake] PathPlanner commands added");
	}
}