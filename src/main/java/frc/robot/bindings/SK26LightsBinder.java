package frc.robot.bindings;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SK26Lights;
import static frc.robot.Ports.OperatorPorts.kAbutton;
import static frc.robot.Ports.OperatorPorts.kBbutton;
import static frc.robot.Ports.OperatorPorts.kYbutton;
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
        if (lightsSubsystem.isEmpty()) {
            return;
        }
        SK26Lights lights = lightsSubsystem.get();

        // Base effects (infinite - persist until changed)
        skBlueWave.onTrue(new InstantCommand(
            () -> lights.requestSKBlueWave()
        ).ignoringDisable(true));
        
        leftBumper.onTrue(new InstantCommand(
            () -> lights.requestLEDWhite()
        ).ignoringDisable(true));

        // Temporary overlay effects (2 second duration - then returns to base)
        kAbutton.button.onTrue(new InstantCommand(
            () -> lights.requestLEDRed(2.0)
        ).ignoringDisable(true));
        
        kBbutton.button.onTrue(new InstantCommand(
            () -> lights.requestLEDBlue(2.0)
        ).ignoringDisable(true));
        
        kYbutton.button.onTrue(new InstantCommand(
            () -> lights.requestLEDGreen(2.0)
        ).ignoringDisable(true));
    }
}