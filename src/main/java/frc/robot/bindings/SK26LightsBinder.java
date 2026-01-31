package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kAbutton;
import static frc.robot.Ports.OperatorPorts.kBbutton;
import static frc.robot.Ports.OperatorPorts.kYbutton;
import static frc.robot.Ports.OperatorPorts.k_BlueWaveTrigger;
import static frc.robot.Ports.OperatorPorts.k_BreathingBlueTrigger;
import static frc.robot.Ports.OperatorPorts.k_LeftBumperTrigger;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SK26Lights;

public class SK26LightsBinder implements CommandBinder {

    private final Optional<SK26Lights> lightsSubsystem;
    private final Trigger blueWaveTrigger;
    private final Trigger breathingBlueTrigger;
    private final Trigger leftBumperTrigger;

    public SK26LightsBinder(Optional<SK26Lights> lightsSubsystem) {
        this.lightsSubsystem = lightsSubsystem;
        this.blueWaveTrigger = k_BlueWaveTrigger.button;
        this.breathingBlueTrigger = k_BreathingBlueTrigger.button;
        this.leftBumperTrigger = k_LeftBumperTrigger.button;
    }

    @Override
    public void bindButtons() {
        if (lightsSubsystem.isEmpty()) {
            return;
        }
        SK26Lights lights = lightsSubsystem.get();

        breathingBlueTrigger.onTrue(lights.setBreathingSKBlue().ignoringDisable(true));
        leftBumperTrigger.onTrue(lights.setSolidWhite().ignoringDisable(true));
        blueWaveTrigger.onTrue(lights.setRainbow().ignoringDisable(true));

        kAbutton.button.onTrue(lights.setSolidRed().ignoringDisable(true));
        kBbutton.button.onTrue(lights.setSolidBlue().ignoringDisable(true));
        kYbutton.button.onTrue(lights.setSolidGreen().ignoringDisable(true));
    }
}