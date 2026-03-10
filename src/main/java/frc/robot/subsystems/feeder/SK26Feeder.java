package frc.robot.subsystems.feeder;

import static frc.robot.Konstants.FeederConstants.kMaxFeederVoltage;
import static frc.robot.Konstants.FeederConstants.kFeederIdleVelocity;
import static frc.robot.Ports.LauncherPorts.kFeederMotor;
import static frc.robot.Ports.Sensors.launcherSensor;

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

import org.littletonrobotics.junction.Logger;

public class SK26Feeder extends SubsystemBase
{
    private final SparkFlex feederMotor;
    private final RelativeEncoder encoder;

    // Ball launch tracking
    private int numBallsLaunched = 0;
    private boolean lastLauncherSensorState = false;

    public SK26Feeder() 
    {
        // ========== Motor Configuration ==========
        feederMotor = new SparkFlex(kFeederMotor.ID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .voltageCompensation(12.0); // Enable voltage compensation for consistent behavior
        feederMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

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

    /**
     * Sets the velocity of the feeder motor in RPS (Rotations Per Second).
     * Internally converts to voltage control.
     * @param velocity The desired velocity in RPS.
     */
    public void setFeederVelocity(double velocity) {
        // Neo Vortex free speed is ~113 RPS (6784 RPM) at 12V
        // Convert RPS to voltage (approximate open-loop)
        double voltage = (velocity / 113.0) * 12.0;
        setFeederVoltage(voltage);
    }

    public Command setFeederVelCommand(double velocityRPS) {
        return run(() -> setFeederVelocity(velocityRPS));
    }

    public Command setFeederVoltageCommand(double voltage) {
        return run(() -> setFeederVoltage(voltage));
    }

    public void idleFeeder() 
    {
        setFeederVelocity(kFeederIdleVelocity);
    }

    /**
     * Feeds fuel by setting the feeder to the feed speed.
     * @param feederFeedRPS The feed speed in RPS.
     */
    public void feedFuel(double feederFeedRPS) 
    {
        setFeederVelocity(feederFeedRPS);
    }

    public double getVoltage() {
        return feederMotor.getAppliedOutput();
    }

    public double getVelocity() {
        return encoder.getVelocity()/60;
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
