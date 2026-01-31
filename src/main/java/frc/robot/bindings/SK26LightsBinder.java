package frc.robot.bindings;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SK26Lights;
import frc.robot.commands.RequestLEDWhite;
import static frc.robot.subsystems.SK26Lights.LightEffect;


// import static frc.robot.Konstants.SystemConstants.kBrownoutTrigger;
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
        if(lightsSubsystem.isEmpty()) {
            return;
        }
        SK26Lights lights = lightsSubsystem.get();

        leftBumper.onTrue(new RequestLEDWhite(lights).ignoringDisable(true));

        // kBrownoutTrigger.onTrue(new InstantCommand(
        //     () -> lights.requestEffect(LightEffect.BROWNOUT)
        // ));
        // kBrownoutTrigger.onFalse(new InstantCommand(
        //     () -> lights.cancelEffect(LightEffect.BROWNOUT)
        // ));
    }
}