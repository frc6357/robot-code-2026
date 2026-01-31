package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.commands.RunLauncherCommand;
import frc.robot.subsystems.SK26Launcher;

import static frc.robot.Konstants.LauncherConstants.kStopLauncher;
import static frc.robot.Konstants.LauncherConstants.kUnJamLauncherPauseTime;
import static frc.robot.Konstants.LauncherConstants.kUnJamLauncherRPS;
import static frc.robot.Konstants.LauncherConstants.kUnJamLauncherRunTime;

/**
 * Command group that repeatedly "unjams" the launcher by alternating the flywheel direction.
 *
 * <p>This sequence is intended to clear stuck game pieces by:
 * <ol>
 *   <li>Running the launcher in reverse for {@link frc.robot.Konstants.LauncherConstants#kUnJamLauncherRunTime}</li>
 *   <li>Stopping for {@link frc.robot.Konstants.LauncherConstants#kUnJamLauncherPauseTime}</li>
 *   <li>Running forward for {@link frc.robot.Konstants.LauncherConstants#kUnJamLauncherRunTime}</li>
 *   <li>Stopping for {@link frc.robot.Konstants.LauncherConstants#kUnJamLauncherPauseTime}</li>
 * </ol>
 *
 * <p>The sequence is wrapped in {@code repeatedly()}, meaning it will continue looping until the
 * command is interrupted (typically when the button is released). When the command ends, it calls
 * {@link frc.robot.subsystems.SK26Launcher#idleLauncher()} via {@code finallyDo(...)} so the launcher
 * returns to its idle setpoint instead of staying stopped or running unintentionally.
 *
 * <p>Requires the {@link frc.robot.subsystems.SK26Launcher} subsystem.
 */
public class LauncherUnJamCommandGroup extends SequentialCommandGroup {

    SK26Launcher launchermotor;
    double UnJamLauncherVelocity;

    /**
     * Creates an unjam command that loops until interrupted.
     *
     * @param launchermotor the launcher subsystem to control
     */
    public LauncherUnJamCommandGroup(SK26Launcher launchermotor) {

        addRequirements(launchermotor);
        //Makes motor rotate in reverse and forward to unjam any stuck game pieces
        addCommands(
            Commands.sequence(
                //Reverses motor
                Commands.runOnce(() -> launchermotor.unJamLauncher(-kUnJamLauncherRPS)),
                Commands.waitSeconds(kUnJamLauncherRunTime),

                //Stops motor
                Commands.runOnce(() -> launchermotor.unJamLauncher(kStopLauncher)),
                Commands.waitSeconds(kUnJamLauncherPauseTime),

                //Runs motor
                Commands.runOnce(() -> launchermotor.unJamLauncher(kUnJamLauncherRPS)),
                Commands.waitSeconds(kUnJamLauncherRunTime),

                //Stops motor
                Commands.runOnce(() -> launchermotor.unJamLauncher(kStopLauncher)),
                Commands.waitSeconds(kUnJamLauncherPauseTime))
                .repeatedly() //makes the unjam command repeat until button is let go
                .finallyDo(() -> launchermotor.idleLauncher()) //stops the launcher once it stops repeating
        );
    }
    
}
