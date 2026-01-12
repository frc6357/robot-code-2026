package frc.robot.subsystems;

import static frc.robot.Ports.LauncherPorts.kLauncherMotor;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SKLauncher extends SubsystemBase {

    public SKLauncher() {
        //initialize motor objects
        SparkFlex motor = new SparkFlex(kLauncherMotor.ID, MotorType.kBrushless);
    }

    public void initialize() {
        
    }
    
}
