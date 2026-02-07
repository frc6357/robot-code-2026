package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kAbutton;
import static frc.robot.Ports.OperatorPorts.kBbutton;
import static frc.robot.Ports.OperatorPorts.kYbutton;
import static frc.robot.Ports.OperatorPorts.kXbutton;
import static frc.robot.Ports.OperatorPorts.k_Start;
import static frc.robot.Ports.OperatorPorts.k_LeftTrigger;
import static frc.robot.Ports.OperatorPorts.k_LeftBumperTrigger;
import static frc.robot.Ports.OperatorPorts.k_RightBumperTrigger;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.lights.SK26Lights;

public class SK26LightsBinder implements CommandBinder {

    private final Optional<SK26Lights> lightsSubsystem;

    public SK26LightsBinder(Optional<SK26Lights> lightsSubsystem) {
        this.lightsSubsystem = lightsSubsystem;
    }

    @Override
    public void bindButtons() {
        if (lightsSubsystem.isEmpty()) {
            return;
        }
        SK26Lights lights = lightsSubsystem.get();
        
        kXbutton.button.onTrue(lights.setOff());
        kAbutton.button.onTrue(lights.setSolidRed());
        kBbutton.button.onTrue(lights.setSolidBlue());
        kYbutton.button.onTrue(lights.setAllianceGradient());
        k_LeftBumperTrigger.button.onTrue(lights.setSolidWhite());
        k_RightBumperTrigger.button.onTrue(lights.setBreathingSKBlue());
        k_Start.button.onTrue(lights.activatePartyMode());
        k_LeftTrigger.button.onTrue(lights.setSKBlueGradient());
    }
}