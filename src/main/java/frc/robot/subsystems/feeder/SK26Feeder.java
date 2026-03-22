package frc.robot.subsystems.feeder;

import static frc.robot.Konstants.FeederConstants.kMaxFeederVoltage;
import static frc.robot.Konstants.FeederConstants.kFeederIdleVoltage;
import static frc.robot.Ports.LauncherPorts.kFeederMotor;
import static frc.robot.Ports.Sensors.launcherSensor;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class SK26Feeder extends SubsystemBase
{
    private final TalonFX feederMotor;
    private final VoltageOut m_voltageRequest = new VoltageOut(0).withEnableFOC(false);

    // Ball launch tracking
    private int numBallsLaunched = 0;
    private boolean lastLauncherSensorState = false;

    public SK26Feeder() 
    {
        // ========== Motor Configuration ==========
        feederMotor = new TalonFX(kFeederMotor.ID);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        feederMotor.getConfigurator().apply(config);
    }

    /**
     * Sets the voltage of the feeder motor directly.
     * Clamped for safety to prevent brownouts.
     * @param voltage The desired voltage (-12 to +12 volts).
     */
    public void setFeederVoltage(double voltage) {
        // Clamp output for safety
        voltage = MathUtil.clamp(voltage, -kMaxFeederVoltage, kMaxFeederVoltage);
        feederMotor.setControl(m_voltageRequest.withOutput(voltage));
    }

    public void idleFeeder() 
    {
        setFeederVoltage(kFeederIdleVoltage);
    }

    public Command idleFeederCommand() {
        return this.runOnce(() -> idleFeeder());
    }

    /**
     * Feeds fuel by setting the feeder to the feed speed.
     * @param feederVoltage The feed speed in RPS.
     */
    public void feedFuel(double feederVoltage) 
    {
        setFeederVoltage(feederVoltage);
    }

    public double getVoltage() {
        return feederMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getVelocity() {
        return feederMotor.getVelocity().getValueAsDouble(); // TalonFX reports velocity in RPS
    }

    /**
     * Returns a command that sets the feeder to the given voltage once.
     * Replaces the standalone FeederFeedCommand.
     *
     * @param voltage The voltage to feed at.
     * @return A command requiring this subsystem.
     */
    public Command feedCommand(double voltage) {
        return this.runEnd(() -> feedFuel(voltage), () -> idleFeeder());
    }

    public Command feedCommand(Supplier<Double> voltageSupplier) {
        return this.runEnd(() -> feedFuel(voltageSupplier.get()), () -> idleFeeder());
    }

    @Override
    public void periodic() {
        checkIfBallLaunched();
        logOutputs();
    }

    /**
     * Checks the launcher sensor for a rising edge (ball passing through).
     * Increments the launched ball count on each detection.
     */
    private void checkIfBallLaunched() {
        boolean isBallPresent = launcherSensor.getIsDetected(true).getValue();

        if (!lastLauncherSensorState && isBallPresent) {
            numBallsLaunched++;
        }
        lastLauncherSensorState = isBallPresent;
    }

    private void logOutputs() {
        Logger.recordOutput("Feeder/Total Balls Launched", numBallsLaunched);
        Logger.recordOutput("Feeder/Current Velocity RPS", getVelocity());
        Logger.recordOutput("Feeder/Current Voltage", getVoltage());
    }
}
