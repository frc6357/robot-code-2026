package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Ports.IndexerPorts.kIndexerMotor;

public class SK26Indexer extends SubsystemBase{
    private final TalonFX indexerMotor;
    TalonFXConfiguration config = new TalonFXConfiguration().withSlot0(
        new Slot0Configs().withKP(50)
    );
    
    private boolean isFeeding = false;

    double targetIndexerSpeed = 0.0;

    public SK26Indexer() 
    {
        indexerMotor = new TalonFX(kIndexerMotor.ID);
        indexerMotor.setNeutralMode(NeutralModeValue.Brake);

        indexerMotor.getConfigurator().apply(config);
    }

    public void feedFuel(double indexerFeedRPS) {
        setIndexerVelocity(indexerFeedRPS);
    }

    /**
     * Sets the velocity of the indexer motor in RPS (Rotations Per Second).
     * @param velocity The desired velocity in RPS.
     */
    public void setIndexerVelocity(double velocity) {
        // In RPS
        targetIndexerSpeed = velocity;
        indexerMotor.setControl(new VelocityVoltage(velocity));
    }

    public Command setIndexerVelCommand(double velocityRPS) {
        return run(() -> setIndexerVelocity(velocityRPS));
    }


    @Override
    public void periodic() {
        SmartDashboard.putData("Indexer", this);
    }

    @Override 
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "Motor Speed (RPS)", 
            () -> indexerMotor.getVelocity().getValueAsDouble(),
            null);

        builder.addBooleanProperty("IsFeedig", () -> isFeeding, null);

        builder.addDoubleProperty(
            "Target Motor Speed (RPS)", 
            () -> targetIndexerSpeed, 
            null);
    }
}