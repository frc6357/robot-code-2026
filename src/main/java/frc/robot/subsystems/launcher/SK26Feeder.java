package frc.robot.subsystems.launcher;

import static frc.robot.Konstants.LauncherConstants.kFeederVoltage;
import static frc.robot.Ports.LauncherPorts.kFeederMotor;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SK26Feeder extends SubsystemBase{

    SparkFlex feedermotor;

    public SK26Feeder() {

        feedermotor = new SparkFlex(kFeederMotor.ID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        feedermotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runFeeder() {
        feedermotor.setVoltage(kFeederVoltage);
    }

    public void stopFeeder() {
        feedermotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        
    }
}
