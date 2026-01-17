package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Ports.IndexerPorts.kIndexerMotor;

public class SK26Indexer extends SubsystemBase{
    private final TalonFX indexerMotor;
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    private boolean isFeeding = false;

    public SK26Indexer() 
    {
        indexerMotor = new TalonFX(kIndexerMotor.ID);
        indexerMotor.setNeutralMode(NeutralModeValue.Brake);

        indexerMotor.getConfigurator().apply(config);
    }

    public void feedFuel(double indexerMotorSpeedRPS) {
        double indexerSpeed = indexerMotorSpeedRPS;
        isFeeding = true;
        setIndexerVelocity(indexerSpeed);
    }
    /**
     * Sets the velocity of the indexer motor in RPS (Rotations Per Second).
     * @param velocity The desired velocity in RPS.
     */
    public void setIndexerVelocity(double velocity) {
        // In RPS
        indexerMotor.setControl(new VelocityVoltage(velocity));
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
    }

    public SK26Indexer get() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'get'");
    }

}