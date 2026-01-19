package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.Konstants.IndexerConstants.kIndexerIdleRPS;
import static frc.robot.Ports.IndexerPorts.kIndexerMotor;

// Subsystem for the SK26 Indexer mechanism
public class SK26Indexer extends SubsystemBase{
    private final TalonFX indexerMotor;

    // Configuration for the TalonFX motor controller
    TalonFXConfiguration config = new TalonFXConfiguration()

    // PID Configuration
    .withSlot0(new Slot0Configs().withKP(50))

    // this essentially sets the motor to a max current supply of 120 amps
    .withCurrentLimits(
        new CurrentLimitsConfigs()

            .withStatorCurrentLimit(Amps.of(120))
            .withStatorCurrentLimitEnable(true)
    )   
    ;

    // String to represent the current status of the indexer
    public String status = "Idle";

    // Target speed for the indexer motor in RPS
    double targetIndexerSpeed = 0.0;

    // Constructor
    public SK26Indexer() 
    {
        indexerMotor = new TalonFX(kIndexerMotor.ID);
        indexerMotor.setNeutralMode(NeutralModeValue.Brake);

        indexerMotor.getConfigurator().apply(config);
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

    /**
     * Idle the indexer by setting it to the predefined idle speed.
     */
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

    // Periodic method to update SmartDashboard with indexer status
    @Override
    public void periodic() {
        SmartDashboard.putData("Indexer", this);
    }

    // Method to initialize Sendable properties for SmartDashboard
    @Override 
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "Motor Speed (RPS)", 
            () -> indexerMotor.getVelocity().getValueAsDouble(),
            null);
        
        builder.addStringProperty(
            "Status", 
            () -> status, 
            null);

        builder.addDoubleProperty(
            "Target Motor Speed (RPS)", 
            () -> targetIndexerSpeed, 
            null);
    }
}