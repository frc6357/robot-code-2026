package frc.robot.subsystems;

import static frc.robot.Ports.LauncherPorts.kAgitatorMotor;
import static frc.robot.Konstants.IndexerConstants.kFeederSpeed;
import static frc.robot.Konstants.IndexerConstants.kFeederStop;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SK26Indexer extends SubsystemBase 
{
    // Motor
    private TalonFX agitatorMotor;

    .withSlot0(new Slot0Configs().withKP(50))

    // this essentially sets the motor to a max current supply of 120 amps
    .withCurrentLimits(
        new CurrentLimitsConfigs()

            .withStatorCurrentLimit(Amps.of(120))
            .withStatorCurrentLimitEnable(true)
    )   
    ;

    public SK26Indexer() 
    {
        agitatorMotor = new TalonFX(kAgitatorMotor.ID);
        configureAgitatorMotor();
    }

    private void configureAgitatorMotor() 
    {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kAgitatorP;
        config.Slot0.kI = kAgitatorI;
        config.Slot0.kD = kAgitatorD;
        config.Slot0.kV = kAgitatorF;

        agitatorMotor.getConfigurator().apply(config);
        agitatorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void feedFuel()
    {
        //TODO in the future, fuel should be fed if the launcher is shooting and the shooter is at the correct speed.
        agitatorMotor.set(kFeederSpeed);
    }
    public void stopFeeding()
    {
        agitatorMotor.set(kFeederStop);
    }

    @Override
    public void periodic() {}
}
