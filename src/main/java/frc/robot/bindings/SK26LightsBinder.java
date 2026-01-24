package frc.robot.bindings;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SK26Lights;
import frc.robot.commands.RequestSKBlueWave;
import frc.robot.commands.RequestLEDWhite;

import static frc.robot.Ports.OperatorPorts.k_BlueWaveTrigger;
import static frc.robot.Ports.OperatorPorts.k_LeftBumperTrigger;

import java.util.Optional;

public class SK26LightsBinder implements CommandBinder {

    private final Optional<SK26Lights> lightsSubsystem;
    private final Trigger skBlueWave;
    private final Trigger leftBumper;

    public SK26LightsBinder(Optional<SK26Lights> lightsSubsystem) {
        this.lightsSubsystem = lightsSubsystem;
        this.skBlueWave = k_BlueWaveTrigger.button;
        this.leftBumper = k_LeftBumperTrigger.button;
    }

    @Override
    public void bindButtons() {
        lightsSubsystem.ifPresent(lights -> {
            skBlueWave.onTrue(new RequestSKBlueWave(lights));
            leftBumper.onTrue(new RequestLEDWhite(lights));
        });
    }
}