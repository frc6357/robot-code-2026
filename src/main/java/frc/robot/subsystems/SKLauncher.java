package frc.robot.subsystems;

import static frc.robot.Ports.LauncherPorts.kLauncherMotor;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SKLauncher extends SubsystemBase {

    //TODO: Moved motor variable declaration outside of constructor since it will be needed in other methods
    //TODO: Changed motor type to TalonFX since we're using krakens
    private TalonFX motor;

    public SKLauncher() {
        //initialize motor objects
        motor = new TalonFX(kLauncherMotor.ID);
    }

    public void initialize() {
        
    }
    
}
