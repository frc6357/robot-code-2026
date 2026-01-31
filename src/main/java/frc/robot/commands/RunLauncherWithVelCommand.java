package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK26Launcher;

/**
 * Command that runs the launcher flywheel to a requested exit velocity (m/s).
 *
 * <p>While scheduled, this command continuously calls
 * {@link frc.robot.subsystems.SK26Launcher#runLauncherExitVel(double, String)} to maintain the
 * target velocity. The command never finishes on its own; it is intended to be used with a
 * trigger/button binding.
 *
 * <p>When the command ends (button released or interrupted), the launcher is set to idle via
 * {@link frc.robot.subsystems.SK26Launcher#idleLauncher()} so it stays spun up for faster
 * subsequent shots.
 *
 * <p>Requires the {@link frc.robot.subsystems.SK26Launcher} subsystem.
 */
public class RunLauncherWithVelCommand extends Command {

    SK26Launcher launchermotor;
    double targetLaunchVelocity;

    /**
     * Creates a command that runs the launcher to a target exit velocity.
     *
     * @param launchermotor the launcher subsystem to control
     * @param targetLaunchVelocity desired exit velocity in meters per second (m/s)
     */
    public RunLauncherWithVelCommand(SK26Launcher launchermotor, double targetLaunchVelocity) {
        this.launchermotor = launchermotor;
        this.targetLaunchVelocity = targetLaunchVelocity;
    }

    @Override
    public void initialize() {
        // no-op
    }

    @Override
    public void execute() {
        launchermotor.runLauncherExitVel(targetLaunchVelocity, "Launching");
    }

    @Override
    public void end(boolean interrupted) {
        //launchermotor.stopLauncher();
        launchermotor.idleLauncher(); //makes the launcher quickly relaunchable
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
