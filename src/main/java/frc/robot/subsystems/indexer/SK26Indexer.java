package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.PathplannerSubsystem;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.Konstants.IndexerConstants.kIndexerIdleRPS;
import static frc.robot.Ports.IndexerPorts.kIndexerMotor;
import static frc.robot.Ports.IndexerPorts.kSpindexerMotor;
import static frc.robot.Ports.Sensors.hopperSensor;
import static frc.robot.Ports.Sensors.launcherSensor;
import static frc.robot.Ports.Sensors.intakeSensor1;
import static frc.robot.Ports.Sensors.intakeSensor2;
import static frc.robot.Konstants.IndexerConstants.kIndexerHeight;

// Subsystem for the SK26 Indexer mechanism
public class SK26Indexer extends SubsystemBase implements PathplannerSubsystem {
    private final TalonFX indexerMotor;
    private final TalonFX spindexerMotor;

    // Configuration for the TalonFX motor controller
    TalonFXConfiguration config = new TalonFXConfiguration()

    // PID Configuration
    .withSlot0(new Slot0Configs().withKP(16.617))

    // this essentially sets the motor to a max current supply of 120 amps
    .withCurrentLimits(
        new CurrentLimitsConfigs()

            .withStatorCurrentLimit(Amps.of(120))
            .withStatorCurrentLimitEnable(true)
    );

    // String to represent the current status of the indexer
    public String status = "Idle";

    // Target speed for the indexer motor in RPS
    double targetIndexerSpeed = 0.0;
    double targetSpindexerSpeed = 0.0;

    //Sensor values
    public int numBallsInIndexer = 0;
    public int totalNumBallsLaunched = 0;
    public boolean lastLauncherSensorState;
    public boolean lastIntakeSensorState1;
    public boolean lastIntakeSensorState2;

    // Constructor
    public SK26Indexer() 
    {
        indexerMotor = new TalonFX(kIndexerMotor.ID);
        spindexerMotor = new TalonFX(kSpindexerMotor.ID);

        indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        spindexerMotor.setNeutralMode(NeutralModeValue.Brake);

        indexerMotor.getConfigurator().apply(config);
        spindexerMotor.getConfigurator().apply(config);
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
     * @param velocity The desired velocity in RPS.
     */
    public void setIndexerVelocity(double velocity) {
        targetIndexerSpeed = velocity;
        indexerMotor.setControl(new VelocityVoltage(velocity));
    }

    public Command setIndexerVelCommand(double velocityRPS) {
        return run(() -> setIndexerVelocity(velocityRPS));
    }

    // -----------------
    // SPINDEXER STUFF
    // -----------------
    public void setSpindexerVelocity(double velocity) {
        targetSpindexerSpeed = velocity;
        spindexerMotor.setControl(new VelocityVoltage(velocity));
    }
    
    public Command setSpindexerVelCommand(double velocityRPS) {
        return run(() -> setSpindexerVelocity(velocityRPS));
    }

    public void idleIndexer() {
        setIsIdle();
        setIndexerVelocity(kIndexerIdleRPS);
        setSpindexerVelocity(kIndexerIdleRPS);
    }

    /**
     * Feeds fuel by setting the indexer to the feed speed.
     * @param indexerFeedRPS The feed speed in RPS.
     */
    public void feedFuel(double indexerFeedRPS) {
        setIsFeeding();
        setIndexerVelocity(indexerFeedRPS);
        setSpindexerVelocity(indexerFeedRPS);
    }
    
    /**
     * Runs the indexer at the speed, specifically for unjamming purposes.
     * @param speed The speed to target in RPS.
     */
    public void unjamIndexer(double speed) {
        setIsUnjamming();
        setIndexerVelocity(speed);
        setSpindexerVelocity(speed);
    }
    
    /**
     * Calculates and returns the fullness of the indexer as a value between 0.0 and 1.0.
     * @return The fullness of the indexer, where 0.0 is empty and 1.0 is full.
     */
    public double getFullness() {
        return kIndexerHeight.minus(getTofDistance()).div(kIndexerHeight).baseUnitMagnitude();
    }

    /**
     * Gets the current distance reading from the CANrange time-of-flight sensor.
     *
     * @return the measured distance as a {@link edu.wpi.first.units.measure.Distance}
     */
    private Distance getTofDistance() {
        return hopperSensor.getDistance().refresh().getValue();
    }

    /**
     * Updates {@link #numBallsInIndexer} when a ball is detected leaving the launcher.
     *
     * <p>This method reads {@code launcherSensor} and looks for a <b>falling edge</b>
     * (sensor state transitions from {@code true} to {@code false}). When that edge is seen,
     * it assumes a ball has just passed the sensor / been launched and decrements
     * {@link #numBallsInIndexer}.
     *
     * <p>Notes/assumptions:
     * <ul>
     *   <li>{@link #lastLauncherSensorState} must be maintained between calls; call this periodically.</li>
     *   <li>The edge direction depends on whether the beam break is active-high or active-low.
     *       If counts are inverted, swap the edge check accordingly.</li>
     * </ul>
     */
    private void checkIfBallLaunched() {
        boolean currentState = launcherSensor.getIsDetected().getValue(); // Read sensor
    
        // Check for falling edge (state changes from true to false)
        if (lastLauncherSensorState && !currentState) {
            numBallsInIndexer--; // decreases how many balls are in the indexer
            totalNumBallsLaunched++; // increases how many total balls have been launched in a match
        }
        lastLauncherSensorState = currentState; // Update last state
    }

    /**
     * Updates {@link #numBallsInIndexer} when a ball is detected entering the robot (intaked).
     *
     * <p>This method reads {@code intakeSensor} and looks for a <b>falling edge</b>
     * (sensor state transitions from {@code true} to {@code false}). When that edge is seen,
     * it assumes a ball has just crossed the intake sensor and increments
     * {@link #numBallsInIndexer}.
     *
     * <p>Notes/assumptions:
     * <ul>
     *   <li>{@link #lastIntakeSensorState1} must be maintained between calls; call this periodically.</li>
     *   <li>The edge direction depends on whether the beam break is active-high or active-low.
     *       If counts are inverted, swap the edge check accordingly.</li>
     *   <li>If the sensor chatters, consider adding a small debounce (WPILib {@code Debouncer}).</li>
     * </ul>
     */
    private void checkIfBallIntaked() {
        boolean currentState1 = intakeSensor1.getIsDetected().getValue(); // Read sensor
        boolean currentState2 = intakeSensor2.getIsDetected().getValue(); // Read second sensor

        // Check for falling edge (state changes from true to false)
        if(lastIntakeSensorState1 && !currentState1) {
            numBallsInIndexer++; //Increases how many balls are in the indexer
        }
        if(lastIntakeSensorState2 && !currentState2) {
            numBallsInIndexer++;
        }
        lastIntakeSensorState1 = currentState1;
        lastIntakeSensorState2 = currentState2;
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("Indexer", this);

        checkIfBallLaunched();
        checkIfBallIntaked();
    }

    @Override 
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "Indexer Motor Speed (RPS)", 
            () -> indexerMotor.getVelocity().getValueAsDouble(),
            null);

        builder.addDoubleProperty(
            "Spindexer Motor Speed (RPS)", 
            () -> spindexerMotor.getVelocity().getValueAsDouble(),
            null);
        
        builder.addStringProperty(
            "Status", 
            () -> status, 
            null);

        builder.addDoubleProperty(
            "Target Indexer Motor Speed (RPS)", 
            () -> targetIndexerSpeed, 
            null);

        builder.addDoubleProperty(
            "Target Spindexer Motor Speed (RPS)", 
            () -> targetIndexerSpeed, 
            null);

        builder.addIntegerProperty(
            "Number of Balls in Indexer",
            () -> numBallsInIndexer,
            null);

        builder.addIntegerProperty(
            "Number of Balls Intaked",
            () -> totalNumBallsLaunched,
            null);
    }


    @Override
    public void addPathPlannerCommands() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addPathPlannerCommands'");
    }
}