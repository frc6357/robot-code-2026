package frc.robot.subsystems.intake;

// Imports from robot
import static frc.robot.Konstants.IntakeConstants.kMaxIntakeVoltage;
import static frc.robot.Konstants.IntakeConstants.kIntakeSupplyCurrentLimit;
import static frc.robot.Konstants.IntakeConstants.kChassisSpeedRollerFF;
import static frc.robot.Konstants.IntakeConstants.kIntakeStationaryVoltage;
import static frc.robot.Konstants.IntakeConstants.kIntakeStatorCurrentLimit;
import static frc.robot.Ports.pickupOBPorts.kIntakeMotor;

import java.util.function.Supplier;

// Imports from Phoenix 6
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.littletonrobotics.junction.Logger;

// Imports from WPILib
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports.DriverPorts;

/**
 * Subsystem for the intake roller motor. Controls the roller that intakes game pieces
 * using simple voltage control. Uses a TalonFX (Kraken X44) motor.
 */
public class SK26IntakeRollers extends SubsystemBase
{
	// Motor
	private final TalonFX intakeMotor;

	// Control request
	private final VoltageOut intakeVoltageControl = new VoltageOut(0);

	// Target voltage for the intake motor
	private double targetVoltage = 0.0;

	public SK26IntakeRollers()
	{
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
		if(!DriverStation.isAutonomousEnabled()){
			DriverPorts.kDriver.setRumble(RumbleType.kBothRumble, 0.6);
		}
	}

	/**
	 * Runs the intake rollers based on the robot's chassis speeds. This is to compensate for
	 * the increasing entrance speed of Fuel as it enters the intake system.
	 * @param robotSpeeds The current speeds of the robot chassis.
	 */
	public static double calculateRollerVoltageFromRobotVel(ChassisSpeeds robotSpeeds) {
		// Calculate the speed of the intake as it approaches Fuel, which fortunately is on the front of the robot
		double forwardSpeed = robotSpeeds.vxMetersPerSecond;

		// Map the forward speed to an intake voltage (this is a simple linear mapping, can be tuned)
		// Unfortunately the signs of the math is a little funky, but I am just working with what is already here.
		double voltage = MathUtil.clamp(kIntakeStationaryVoltage - (forwardSpeed * kChassisSpeedRollerFF), -kMaxIntakeVoltage, kIntakeStationaryVoltage);
		
		return voltage;
	}

	public Command runBasedOnRobotSpeedCommand(Supplier<ChassisSpeeds> robotSpeeds) {
		return this.runEnd(() -> setIntakeVoltage(calculateRollerVoltageFromRobotVel(robotSpeeds.get())), () -> stopIntake());
	}

	/**
	 * Command factory that runs the rollers at the given voltage and stops on end.
	 */
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
		if(!DriverStation.isAutonomousEnabled()){
			DriverPorts.kDriver.setRumble(RumbleType.kBothRumble, 0.0);
		}
	}

	@Override
	public void periodic()
	{
		logOutputs();
	}

	private void logOutputs()
	{
		Logger.recordOutput("Intake/Roller Target Voltage", targetVoltage);
		Logger.recordOutput("Intake/Roller Velocity (RPS)", intakeMotor.getVelocity().getValueAsDouble());
	}
}
