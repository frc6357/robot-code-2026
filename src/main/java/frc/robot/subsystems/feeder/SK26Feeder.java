package frc.robot.subsystems.feeder;

import static frc.robot.Konstants.FeederConstants.kMaxFeederVoltage;
import static frc.robot.Konstants.FeederConstants.kFeederIdleVoltage;
import static frc.robot.Ports.LauncherPorts.kFeederFollowerMotor;
import static frc.robot.Ports.LauncherPorts.kFeederMotor;
import static frc.robot.Ports.Sensors.launcherSensor;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Inline command factory methods replace standalone FeederFeedCommand

import org.littletonrobotics.junction.Logger;

public class SK26Feeder extends SubsystemBase
{
    private final SparkFlex feederMotor;
    private final RelativeEncoder encoder;
    private final SparkFlex feederFollower;

    // Ball launch tracking
    private int numBallsLaunched = 0;
    private boolean lastLauncherSensorState = false;

    public SK26Feeder() 
    {
        // ========== Motor Configuration ==========
        feederMotor = new SparkFlex(kFeederMotor.ID, MotorType.kBrushless);
        feederFollower = new SparkFlex(kFeederFollowerMotor.ID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .voltageCompensation(12.0); // Enable voltage compensation for consistent behavior
        feederMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        feederFollower.configure(config.follow(feederMotor).inverted(false), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        encoder = feederMotor.getEncoder();
    }

    /**
     * Sets the voltage of the feeder motor directly.
     * Clamped for safety to prevent brownouts.
     * @param voltage The desired voltage (-12 to +12 volts).
     */
    public void setFeederVoltage(double voltage) {
        // Clamp output for safety
        voltage = MathUtil.clamp(voltage, -kMaxFeederVoltage, kMaxFeederVoltage);
        feederMotor.setVoltage(voltage);
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
        return feederMotor.getAppliedOutput();
    }

    public double getVelocity() {
        return encoder.getVelocity()/60;
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
