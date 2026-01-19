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
import static frc.robot.Ports.IndexerPorts.kSpindexerMotor;

// Subsystem for the SK26 Indexer mechanism
public class SK26Indexer extends SubsystemBase{
    private final TalonFX indexerMotor;
    private final TalonFX spindexerMotor;

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
    double targetSpindexerSpeed = 0.0;

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

    // SPINDEXER STUFF
    // -----------------

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
    
    


    @Override
    public void periodic() {
        SmartDashboard.putData("Indexer", this);
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
    }
}