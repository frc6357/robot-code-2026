package frc.robot.subsystems.indexer;

// Imports from robot
import static frc.robot.Konstants.IndexerConstants.kIndexerIdleVoltage;
import static frc.robot.Ports.IndexerPorts.kIndexerMotor;

import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;

/**
 * Indexer subsystem using CANrange sensor for gamepiece recognition
 * Controls the indexer motor (Neo Vortex) using voltage control to help avoid brownouts.
 * Tracks the number of balls in the indexer using sensors.
 */
public class SK26Indexer extends SubsystemBase
{

    Pref<Double> indexerVoltage = SKPreferences.attach("Indexer Voltage", 0.0)
		.onChange((newValue) -> updateVoltage());

    private void updateVoltage() 
    {
        targetVoltage = indexerVoltage.get();
        indexerMotor.setVoltage(targetVoltage);
    }

    // Motor
    private final SparkFlex indexerMotor;

    // Built-in motor encoder
    private final RelativeEncoder indexerEncoder;

    // Target voltage for the indexer motor
    private double targetVoltage = 0.0;

    // Ball count tracked by external events (intake sensor, feeder notification, etc.)
    private int numBallsInIndexer = 0;

    // Display status
    private IndexerStatus status = IndexerStatus.IDLE;
    // private boolean lastIntakeSensorState = false;

    /** Possible operational states of the indexer, for telemetry/logging. */
    public enum IndexerStatus {
        IDLE,
        FEEDING,
        UNJAMMING
    }

    public SK26Indexer() 
    {
        System.out.println("Indexer initialized");
        // ========== Motor Configuration ==========
        indexerMotor = new SparkFlex(kIndexerMotor.ID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .voltageCompensation(12.0); // Enable voltage compensation for consistent behavior
        indexerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // ========== Encoder Configuration ==========
        indexerEncoder = indexerMotor.getEncoder();
    }

    /** Sets the current status of the indexer (for telemetry). Should be called by commands. */
    public void setStatus(IndexerStatus status) {
        this.status = status;
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

    public void idleIndexer() 
    {
        setIndexerVoltage(kIndexerIdleVoltage);
    }

    public Command idleIndexerCommand() {
        return this.runOnce(() -> {
            setStatus(IndexerStatus.IDLE);
            idleIndexer();
        });
    }

    /**
     * Feeds fuel by setting the indexer to the feed speed.
     * @param indexerFeedRPS The feed speed in RPS.
     */
    public void feedFuel(double voltage) 
    {
        setIndexerVoltage(voltage);
    }

    /**
     * Runs the indexer at the speed, specifically for unjamming purposes.
     * @param speed The speed to target in RPS.
     */
    public void unjamIndexer(double voltage) 
    {
        setIndexerVoltage(voltage);
    }

    /** Returns the current tracked ball count in the indexer. */
    public int getNumBalls() 
    {
        return numBallsInIndexer;
    }

    /** Sets the tracked ball count (e.g. after a reset or manual correction). */
    public void setNumBalls(int count) 
    {
        numBallsInIndexer = count;
    }

    /** Call when a ball enters the indexer (e.g. from the intake sensor). */
    public void incrementBallCount() 
    {
        numBallsInIndexer++;
    }

    /** Call when a ball leaves the indexer (e.g. notified by the feeder after launch). */
    public void decrementBallCount() {
        numBallsInIndexer = Math.max(0, numBallsInIndexer - 1);
    }

    /**
     * Returns a command that feeds the indexer at the given voltage.
     * The command sets status to FEEDING on start and resets to IDLE on end.
     * Replaces the standalone IndexerFeedCommand.
     *
     * @param voltage The voltage to feed at.
     * @return A command requiring this subsystem.
     */
    public Command feedCommand(double voltage) {
        return this.startEnd(
            () -> {
                setStatus(IndexerStatus.FEEDING);
                feedFuel(voltage);
            },
            () -> {
                setStatus(IndexerStatus.IDLE);
                idleIndexer();
            }
        );
    }

    public Command feedCommand(Supplier<Double> voltageSupplier) {
        return this.startEnd(
            () -> {
                setStatus(IndexerStatus.FEEDING);
                feedFuel(voltageSupplier.get());
            },
            () -> {
                setStatus(IndexerStatus.IDLE);
                idleIndexer();
            }
        );
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
        // checkIfBallIntaked();

        logOutputs();
    }

    private void logOutputs() {
        double velocityRPM = indexerEncoder.getVelocity();
        Logger.recordOutput("Indexer/Applied Output", indexerMotor.getAppliedOutput());
        Logger.recordOutput("Indexer/Actual Velocity RPM", velocityRPM);
        Logger.recordOutput("Indexer/Output Current", indexerMotor.getOutputCurrent());
        Logger.recordOutput("Indexer/Bus Voltage", indexerMotor.getBusVoltage());
        Logger.recordOutput("Indexer/Motor Speed (RPS)", velocityRPM / 60.0);
        Logger.recordOutput("Indexer/Status", status.toString());
        Logger.recordOutput("Indexer/Target Voltage (V)", targetVoltage);
        Logger.recordOutput("Indexer/Balls In Indexer", numBallsInIndexer);
    }
}