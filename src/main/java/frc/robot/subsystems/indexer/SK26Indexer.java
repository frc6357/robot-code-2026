package frc.robot.subsystems.indexer;

// Imports from robot
import static frc.robot.Konstants.IndexerConstants.kIndexerIdleSpeed;
import static frc.robot.Ports.IndexerPorts.kIndexerMotor;
import static frc.robot.Ports.Sensors.tofSensor;
import static frc.robot.Ports.Sensors.launcherSensor;
import static frc.robot.Konstants.IndexerConstants.kIndexerHeight;
import static frc.robot.Konstants.IndexerConstants.kMaxIndexerVoltage;

// Imports from REV
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import org.littletonrobotics.junction.Logger;

// Imports from WPILib
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Indexer subsystem using CANrange sensor for gamepiece recognition
 * Controls the indexer motor (Neo Vortex) using voltage control to help avoid brownouts.
 * Tracks the number of balls in the indexer using sensors.
 */
public class SK26Indexer extends SubsystemBase
{
    // Motor
    private final SparkFlex indexerMotor;

    // Built-in motor encoder
    private final RelativeEncoder indexerEncoder;

    // String to represent the current status of the indexer
    public String status = "Idle";

    // Target voltage for the indexer motor
    private double targetVoltage = 0.0;

    // Sensor values
    public int numBallsInIndexer = 0;
    public int totalNumBallsLaunched = 0;
    private boolean lastLauncherSensorState = false;
    // private boolean lastIntakeSensorState = false;

    public SK26Indexer() 
    {
        // ========== Motor Configuration ==========
        indexerMotor = new SparkFlex(kIndexerMotor.ID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .voltageCompensation(12.0); // Enable voltage compensation for consistent behavior
        indexerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // ========== Encoder Configuration ==========
        indexerEncoder = indexerMotor.getEncoder();
    }

    public void setIsIdle() {
        status = "Idle";
    }

    public void setIsUnjamming() {
        status = "Unjamming";
    }

    public void setIsFeeding() {
        status = "Feeding";
    }

    /**
     * Sets the voltage of the indexer motor directly.
     * Clamped for safety to prevent brownouts.
     * @param voltage The desired voltage (-12 to +12 volts).
     */
    public void setIndexerVoltage(double voltage) {
        // Clamp output for safety
        voltage = MathUtil.clamp(voltage, -kMaxIndexerVoltage, kMaxIndexerVoltage);
        targetVoltage = voltage;
        indexerMotor.setVoltage(voltage);
    }

    /**
     * Sets the velocity of the indexer motor in RPS (Rotations Per Second).
     * Internally converts to voltage control.
     * @param velocity The desired velocity in RPS.
     */
    public void setIndexerVelocity(double velocity) {
        // Neo Vortex free speed is ~113 RPS (6784 RPM) at 12V
        // Convert RPS to voltage (approximate open-loop)
        double voltage = (velocity / 113.0) * 12.0;
        setIndexerVoltage(voltage);
    }

    public Command setIndexerVelCommand(double velocityRPS) {
        return run(() -> setIndexerVelocity(velocityRPS));
    }

    public Command setIndexerVoltageCommand(double voltage) {
        return run(() -> setIndexerVoltage(voltage));
    }

    public void idleIndexer() {
        setIsIdle();
        setIndexerVelocity(kIndexerIdleSpeed);
    }

    /**
     * Feeds fuel by setting the indexer to the feed speed.
     * @param indexerFeedRPS The feed speed in RPS.
     */
    public void feedFuel(double indexerFeedRPS) {
        setIsFeeding(); // TODO This should eventually be implemented in the command itself.
        setIndexerVelocity(indexerFeedRPS);
    }

    /**
     * Runs the indexer at the speed, specifically for unjamming purposes.
     * @param speed The speed to target in RPS.
     */
    public void unjamIndexer(double speed) {
        setIsUnjamming();
        setIndexerVelocity(speed);
    }

    public double getFullness() {
        return kIndexerHeight.minus(getTofDistance()).div(kIndexerHeight).baseUnitMagnitude();
    }

    private Distance getTofDistance() {
        return tofSensor.getDistance().refresh().getValue();
    }

    private void checkIfBallLaunched() {
        boolean isBallPresent = launcherSensor.getIsDetected(true).getValue();

        if (lastLauncherSensorState == false && isBallPresent) {
            numBallsInIndexer--;
            totalNumBallsLaunched++;
        }
        lastLauncherSensorState = isBallPresent;
    }

    // private void checkIfBallIntaked() {
    //     boolean currentState = intakeSensor.get();

    //     if (lastIntakeSensorState && !currentState) {
    //         numBallsInIndexer++;
    //     }
    //     lastIntakeSensorState = currentState;
    // }

    @Override
    public void periodic() 
    {
        checkIfBallLaunched();
        // checkIfBallIntaked();

        logOutputs();
    }

    private void logOutputs() {
        Logger.recordOutput("Indexer/Applied Output", indexerMotor.getAppliedOutput());
        Logger.recordOutput("Indexer/Actual Velocity RPM", indexerEncoder.getVelocity());
        Logger.recordOutput("Indexer/Output Current", indexerMotor.getOutputCurrent());
        Logger.recordOutput("Indexer/Bus Voltage", indexerMotor.getBusVoltage());
        Logger.recordOutput("Indexer/Motor Speed (RPS)", indexerEncoder.getVelocity() / 60.0);
        Logger.recordOutput("Indexer/Status", status);
        Logger.recordOutput("Indexer/Target Voltage (V)", targetVoltage);
        Logger.recordOutput("Indexer/Balls In Indexer", numBallsInIndexer);
        Logger.recordOutput("Indexer/Total Balls Launched", totalNumBallsLaunched);
    }
}