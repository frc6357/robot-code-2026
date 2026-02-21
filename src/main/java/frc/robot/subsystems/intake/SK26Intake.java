package frc.robot.subsystems.intake;

// Imports from robot
import static frc.robot.Konstants.pickupOBConstants.kPositionerMotorMinPosition;
import static frc.robot.Konstants.pickupOBConstants.kMaxIntakeVoltage;
import static frc.robot.Ports.pickupOBPorts.kEaterMotor;
import static frc.robot.Ports.pickupOBPorts.kPositionerMotor;
import frc.lib.subsystems.PathplannerSubsystem;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;

// Imports from REV
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

// Imports from WPILib
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;

/**
 * Subsystem for the pickup mechanism of the robot, which includes a positioner motor and an intake motor.
 * The positioner motor is responsible for moving the pickup mechanism to the correct position using PID control,
 * while the intake motor is responsible for intaking game pieces.
 */
public class SK26Intake extends SubsystemBase implements PathplannerSubsystem 
{
	// Motors
	SparkFlex positionerMotor;
	SparkFlex intakeMotor;
	SparkFlexConfig positionerConfig;
	SparkFlexConfig intakeConfig;

	// Encoders
	RelativeEncoder positionerEncoder;
	RelativeEncoder intakeEncoder;

	// PID Control
	SparkClosedLoopController positionerPID;
	double motorCurrentPosition;
	double motorTargetPosition;

	// Target voltage for the intake motor
    double targetVoltage = 0.0;

	// Preferences for tuning
	final Pref<Double> positionerKp = SKPreferences.attach("positionerKp", 0.1)
		.onChange((newValue) -> reconfigurePositioner());
	final Pref<Double> positionerKi = SKPreferences.attach("positionerKi", 0.0)
		.onChange((newValue) -> reconfigurePositioner());
	final Pref<Double> positionerKd = SKPreferences.attach("positionerKd", 0.0)
		.onChange((newValue) -> reconfigurePositioner());

	private void reconfigurePositioner() 
	{
		positionerConfig.closedLoop
			.p(positionerKp.get())
		 	.i(positionerKi.get())
			.d(positionerKd.get());
		positionerMotor.configure(positionerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	public SK26Intake() 
	{
		// ========== Positioner Motor Configuration ==========
		positionerMotor = new SparkFlex(kPositionerMotor.ID, MotorType.kBrushless);
		positionerConfig = new SparkFlexConfig();
		positionerConfig.closedLoop
			.p(positionerKp.get())
			.i(positionerKi.get())
			.d(positionerKd.get())
			.outputRange(-1, 1)
			.maxMotion
			.maxAcceleration(150)
			.allowedProfileError(0.5);
		positionerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60);
		positionerMotor.configure(positionerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		// ========== Intake Motor Configuration ==========
		intakeMotor = new SparkFlex(kEaterMotor.ID, MotorType.kBrushless);
		intakeConfig = new SparkFlexConfig();
		intakeConfig
			.idleMode(IdleMode.kCoast)
			.smartCurrentLimit(40)
			.voltageCompensation(12.0);
		intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		motorCurrentPosition = kPositionerMotorMinPosition;
		motorTargetPosition = kPositionerMotorMinPosition;

		// ========== Encoder Configuration ==========
		positionerEncoder = positionerMotor.getEncoder();
		intakeEncoder = intakeMotor.getEncoder();
		positionerEncoder.setPosition(0);

		// ========== PID Configuration ==========
		positionerPID = positionerMotor.getClosedLoopController();

		SmartDashboard.putData("SK26PickupOB", this);
	}

	/**
	 * Sets the target position for the positioner motor and updates the PID controller's setpoint.
	 * @param targetPosition
	 */
	public void setTargetPosition(double targetPosition) 
	{
		motorTargetPosition = targetPosition;
		positionerPID.setSetpoint(motorTargetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
	}

	/**
	 * Returns the current position of the positioner motor from the encoder.
	 */
	public double getCurrentPosition() 
	{
		return positionerEncoder.getPosition();
	}

	/**
	 * Returns the current target position for the positioner motor.
	 */
	public double getTargetPosition() 
	{
		return motorTargetPosition;
	}

	/**
	 * Returns true if the positioner motor is within 0.5 units of the target position, indicating it has reached its target.
	 */
	public boolean isPositionerAtTarget() 
	{
		return Math.abs(getTargetPosition() - getCurrentPosition()) < 0.5;
	}

	/**
	 * Runs the positioner motor towards the maximum position by setting the target position.
	 */
	public void runPositionerMotor(double position) 
	{
		setTargetPosition(position);
	}

	public void setIntakeVoltage(double voltage) 
	{
        voltage = MathUtil.clamp(voltage, -kMaxIntakeVoltage, kMaxIntakeVoltage);
        targetVoltage = voltage;
        intakeMotor.setVoltage(voltage);
    }

	public void setIntakeVelocity(double velocity) 
	{
        // Neo Vortex free speed is ~113 RPS (6784 RPM) at 12V
        // Convert RPS to voltage (approximate open-loop)
        double voltage = (velocity / 113.0) * 12.0;
        setIntakeVoltage(voltage);
    }

	public void runIntakeMotor(double speed) 
	{
		setIntakeVelocity(speed);
	}

	@Override
	public void periodic() {}

	@Override
	public void initSendable(SendableBuilder builder) 
	{
		builder.addDoubleProperty("Positioner Position", this::getCurrentPosition, null);
		builder.addDoubleProperty("Positioner Target Position", this::getTargetPosition, null);
		builder.addBooleanProperty("Positioner At Target", this::isPositionerAtTarget, null);
		builder.addDoubleProperty("Intake Velocity (RPMs)", intakeMotor::get, null);
	}

	@Override
	public void addPathPlannerCommands() 
	{
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'addPathPlannerCommands'");
	}
}