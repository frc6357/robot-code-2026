package frc.robot.commands.commandGroups;

import static frc.robot.Konstants.LightsConstants.kLightPulseSpeed;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SK26Lights;

public class BlinkLightsCommandGroup extends SequentialCommandGroup {

    SK26Lights lights;

    public BlinkLightsCommandGroup(SK26Lights lights, int[] rgb) {

        addRequirements(lights);
        addCommands(
            Commands.sequence(
                //Turns on light
                Commands.runOnce(() -> lights.turnOnLights(rgb)),
                Commands.waitSeconds(kLightPulseSpeed),

                //Turns off light
                Commands.runOnce(() -> lights.turnOffLights()),
                Commands.waitSeconds(kLightPulseSpeed))
                .repeatedly()
                .finallyDo(() -> lights.turnOffLights())
        );
    }
    
}