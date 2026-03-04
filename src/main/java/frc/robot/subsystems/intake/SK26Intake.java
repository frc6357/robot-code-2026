package frc.robot.subsystems.intake;

// Imports from robot
import static frc.robot.Konstants.IntakeConstants.kPositionerMotorMinPosition;
import static frc.robot.Konstants.IntakeConstants.kMaxIntakeVoltage;
import static frc.robot.Konstants.IntakeConstants.IntakePosition;
import static frc.robot.Ports.pickupOBPorts.kIntakeMotor;
import static frc.robot.Ports.pickupOBPorts.kPositionerMotor;
import static frc.robot.Ports.pickupOBPorts.kPositionerFollowerMotor;
import frc.lib.subsystems.PathplannerSubsystem;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;

// Imports from Phoenix 6
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import org.littletonrobotics.junction.Logger;

// Imports from WPILib
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;

/**
 * Subsystem for the pickup mechanism of the robot, which includes a positioner motor and an intake motor.
 * The positioner motor is responsible for moving the pickup mechanism to the correct position using PID control,
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
	private final PositionVoltage positionControl = new PositionVoltage(0).withSlot(0);
	private final VoltageOut intakeVoltageControl = new VoltageOut(0);
	private final Follower followerControl;

	// Position tracking
	private double motorTargetPosition;

	// Target voltage for the intake motor
	private double targetVoltage = 0.0;

	// Preferences for tuning
	final Pref<Double> positionerKp = SKPreferences.attach("Intake/positionerKp", 12.0)
		.onChange((newValue) -> reconfigurePositioner());
	final Pref<Double> positionerKi = SKPreferences.attach("Intake/positionerKi", 0.0)
		.onChange((newValue) -> reconfigurePositioner());
	final Pref<Double> positionerKd = SKPreferences.attach("Intake/positionerKd", 0.0)
		.onChange((newValue) -> reconfigurePositioner());

	private void reconfigurePositioner() 
	{
		Slot0Configs slot0 = new Slot0Configs();
		slot0.kP = positionerKp.get();
		slot0.kI = positionerKi.get();
		slot0.kD = positionerKd.get();
		positionerMotor.getConfigurator().apply(slot0);
	}

	public SK26Intake() 
	{
		// ========== Positioner Motor Configuration ==========
		positionerMotor = new TalonFX(kPositionerMotor.ID);
		TalonFXConfiguration positionerConfig = new TalonFXConfiguration();
		
		// PID configuration for position control
		positionerConfig.Slot0.kP = positionerKp.get();
		positionerConfig.Slot0.kI = positionerKi.get();
		positionerConfig.Slot0.kD = positionerKd.get();
		
		// Motor output configuration
		positionerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		
		// Current limits
		positionerConfig.CurrentLimits.SupplyCurrentLimit = 60;
		positionerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		positionerConfig.CurrentLimits.StatorCurrentLimit = 80;
		positionerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		
		positionerMotor.getConfigurator().apply(positionerConfig);
		positionerMotor.setPosition(0); // Zero the encoder on startup

		// ========== Positioner Follower Motor Configuration ==========
		positionerFollowerMotor = new TalonFX(kPositionerFollowerMotor.ID);
		TalonFXConfiguration followerConfig = new TalonFXConfiguration();

		followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		followerConfig.CurrentLimits.SupplyCurrentLimit = 60;
		followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		followerConfig.CurrentLimits.StatorCurrentLimit = 80;
		followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		
		// Set up follower control (Opposed = opposite direction from master)
		positionerFollowerMotor.getConfigurator().apply(followerConfig);
		followerControl = new Follower(positionerMotor.getDeviceID(), MotorAlignmentValue.Opposed);
		positionerFollowerMotor.setControl(followerControl);

		// ========== Intake Motor Configuration ==========
		intakeMotor = new TalonFX(kIntakeMotor.ID);
		TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
		intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		intakeConfig.CurrentLimits.SupplyCurrentLimit = 40;
		intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		intakeConfig.CurrentLimits.StatorCurrentLimit = 60;
		intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		intakeMotor.getConfigurator().apply(intakeConfig);

		// Initialize position tracking
		motorTargetPosition = kPositionerMotorMinPosition;
	}

	/**
	 * Sets the target position for the positioner motor and updates the PID controller's setpoint.
	 * @param targetPosition Target position in rotations
	 */
	public void setTargetPosition(double targetPosition) 
	{
		motorTargetPosition = targetPosition;
		positionerMotor.setControl(positionControl.withPosition(targetPosition));
	}

	/**
	 * Returns the current position of the positioner motor from the encoder.
	 * @return Current position in rotations
	 */
	public double getCurrentPosition() 
	{
		return positionerMotor.getPosition().getValueAsDouble();
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
	 * Returns true if the positioner motor is within 0.5 rotations of the target position.
	 */
	public boolean isPositionerAtTarget() 
	{
		return Math.abs(getTargetPosition() - getCurrentPosition()) < 0.5;
	}

	/**
	 * Runs the positioner motor towards the specified position.
	 * @param position Target position in rotations
	 */
	public void setPositionerPosition(double position) 
	{
		setTargetPosition(position);
	}

	public void setPositionerPosition(IntakePosition angle) 
    {
        setPositionerPosition(angle.angle);
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

	/**
	 * Sets the intake motor velocity by converting to voltage.
	 * @param velocity Velocity in RPS (rotations per second)
	 */
	public void setIntakeVelocity(double velocity) 
	{
		// Kraken X44 free speed is ~100 RPS (6000 RPM) at 12V
		// Convert RPS to voltage (approximate open-loop)
		double voltage = (velocity / 100.0) * 12.0;
		setIntakeVoltage(voltage);
	}

	/**
	 * Runs the intake motor at the specified speed.
	 * @param speed Speed in RPS
	 */
	public void runIntakeMotor(double speed) 
	{
		setIntakeVelocity(speed);
	}

	/**
	 * Stops the intake motor.
	 */
	public void stopIntake()
	{
		setIntakeVoltage(0);
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
		// Continuously apply follower control to ensure follower stays synced
		positionerFollowerMotor.setControl(followerControl);

		logOutputs();
	}

	private void logOutputs()
	{
		Logger.recordOutput("Intake/Positioner Position", getCurrentPosition());
		Logger.recordOutput("Intake/Positioner Target Position", getTargetPosition());
		Logger.recordOutput("Intake/Positioner At Target", isPositionerAtTarget());
		Logger.recordOutput("Intake/Intake Voltage", targetVoltage);
		Logger.recordOutput("Intake/Intake Velocity (RPS)", intakeMotor.getVelocity().getValueAsDouble());
	}

	@Override
	public void addPathPlannerCommands() 
	{
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'addPathPlannerCommands'");
	}
}