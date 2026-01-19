package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.commands.RunLauncherCommand;
import frc.robot.subsystems.SK26Launcher;

import static frc.robot.Konstants.LauncherConstants.kStopLauncher;
import static frc.robot.Konstants.LauncherConstants.kUnJamLauncherVelocity;
import static frc.robot.Konstants.LauncherConstants.kUnJamLauncherWaitTime;

public class LauncherUnJamCommandGroup extends SequentialCommandGroup {

    SK26Launcher launchermotor;
    double UnJamLauncherVelocity;

    public LauncherUnJamCommandGroup(SK26Launcher launchermotor) {

        addRequirements(launchermotor);
        addCommands(
            Commands.sequence(
                //Reverses motor
                Commands.runOnce(() -> launchermotor.unJamLauncher(-kUnJamLauncherVelocity)),
                Commands.waitSeconds(kUnJamLauncherWaitTime),

                //Stops motor
                Commands.runOnce(() -> launchermotor.unJamLauncher(kStopLauncher)),
                Commands.waitSeconds(kUnJamLauncherWaitTime),

                //Runs motor
                Commands.runOnce(() -> launchermotor.unJamLauncher(kUnJamLauncherVelocity)),
                Commands.waitSeconds(kUnJamLauncherWaitTime),

                //Stops motor
                Commands.runOnce(() -> launchermotor.unJamLauncher(kStopLauncher)),
                Commands.waitSeconds(kUnJamLauncherWaitTime))
                .repeatedly() //makes the unjam command repeat until button is let go
                .finallyDo(() -> launchermotor.stopLauncher()) //stops the launcher once it stops repeating
        );
    }
    
}
