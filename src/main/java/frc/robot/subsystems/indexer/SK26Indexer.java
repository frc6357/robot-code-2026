package frc.robot.subsystems.indexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Konstants.IndexerConstants.kIndexerIdleRPS;
import static frc.robot.Ports.IndexerPorts.kIndexerMotor;
import static frc.robot.Ports.Sensors.tofSensor;
import static frc.robot.Ports.Sensors.launcherSensor;
import static frc.robot.Ports.Sensors.intakeSensor;
import static frc.robot.Konstants.IndexerConstants.kIndexerHeight;

// Subsystem for the SK26 Indexer mechanism
public class SK26Indexer extends SubsystemBase {
    private final SparkFlex indexerMotor;
    // private final SparkClosedLoopController closedLoopController;
    private final RelativeEncoder encoder;

    // String to represent the current status of the indexer
    public String status = "Idle";

    // Target speed for the indexer motor in RPS
    private double targetIndexerSpeed = 0.0;

    // Sensor values
    public int numBallsInIndexer = 0;
    public int totalNumBallsLaunched = 0;
    private boolean lastLauncherSensorState = false;
    private boolean lastIntakeSensorState = false;

    // Constructor
    @SuppressWarnings("removal")
    public SK26Indexer() {
        indexerMotor = new SparkFlex(kIndexerMotor.ID, MotorType.kBrushless);
        // closedLoopController = indexerMotor.getClosedLoopController();
        encoder = indexerMotor.getEncoder();

        // Configure the SparkFlex for Neo Vortex
        SparkFlexConfig config = new SparkFlexConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        
        config.closedLoop
            .p(0.001)
            .i(0.0)
            .d(0.0);

        // Apply configuration
        indexerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
        System.out.println("SK26Indexer initialized with CAN ID: " + kIndexerMotor.ID);
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
     * Sets the velocity of the indexer motor in RPS (Rotations Per Second).
     * Uses duty cycle for testing - switch to velocity control once working.
     * @param velocity The desired velocity in RPS.
     */
    public void setIndexerVelocity(double velocity) {
        targetIndexerSpeed = velocity;
        
        // TESTING: Use simple duty cycle instead of closed-loop
        // Neo Vortex free speed is ~113 RPS (6784 RPM)
        double dutyCycle = velocity / 113.0;
        indexerMotor.set(dutyCycle);
        
        System.out.println("Indexer set to: " + velocity + " RPS (duty cycle: " + dutyCycle + ")");
        
        // UNCOMMENT THIS for closed-loop velocity control after testing:
        // closedLoopController.setSetpoint(velocity * 60.0, ControlType.kVelocity);
    }

    public Command setIndexerVelCommand(double velocityRPS) {
        return run(() -> setIndexerVelocity(velocityRPS));
    }

    public void idleIndexer() {
        setIsIdle();
        setIndexerVelocity(kIndexerIdleRPS);
    }

    /**
     * Feeds fuel by setting the indexer to the feed speed.
     * @param indexerFeedRPS The feed speed in RPS.
     */
    public void feedFuel(double indexerFeedRPS) {
        setIsFeeding();
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
        boolean currentState = launcherSensor.get();

        if (lastLauncherSensorState && !currentState) {
            numBallsInIndexer--;
            totalNumBallsLaunched++;
        }
        lastLauncherSensorState = currentState;
    }

    private void checkIfBallIntaked() {
        boolean currentState = intakeSensor.get();

        if (lastIntakeSensorState && !currentState) {
            numBallsInIndexer++;
        }
        lastIntakeSensorState = currentState;
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("Indexer", this);
        
        // Debug info
        SmartDashboard.putNumber("Indexer/Applied Output", indexerMotor.getAppliedOutput());
        SmartDashboard.putNumber("Indexer/Actual Velocity RPM", encoder.getVelocity());
        SmartDashboard.putNumber("Indexer/Output Current", indexerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Indexer/Bus Voltage", indexerMotor.getBusVoltage());

        checkIfBallLaunched();
        checkIfBallIntaked();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "Indexer Motor Speed (RPS)",
            () -> encoder.getVelocity() / 60.0,
            null);

        builder.addStringProperty(
            "Status",
            () -> status,
            null);

        builder.addDoubleProperty(
            "Target Indexer Motor Speed (RPS)",
            () -> targetIndexerSpeed,
            null);
        
        builder.addIntegerProperty(
            "Balls In Indexer",
            () -> numBallsInIndexer,
            null);
        
        builder.addIntegerProperty(
            "Total Balls Launched",
            () -> totalNumBallsLaunched,
            null);
    }
}