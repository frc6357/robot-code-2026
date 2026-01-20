package frc.robot.bindings;

import frc.robot.subsystems.SK26Lights;
import frc.robot.commands.TurnOnLights;

import static frc.robot.Konstants.LightsConstants.krgbAuto;
import static frc.robot.Ports.OperatorPorts.kIntake;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SK26LightsBinder implements CommandBinder{

    Optional<SK26Lights> lightsSubsystem;

    Trigger TurnOn;

    public SK26LightsBinder(Optional<SK26Lights> lightsSubsystem){
        this.lightsSubsystem = lightsSubsystem;
        this.TurnOn = kIntake.button;
    }

    @Override
    public void bindButtons() {
       
        if(lightsSubsystem.isPresent()) {

            SK26Lights lights = lightsSubsystem.get();

            TurnOn.whileTrue(new TurnOnLights(lights, krgbAuto));
        }
    }
    
}
