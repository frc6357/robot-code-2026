package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK26Launcher;

/**
 * Command that runs the launcher flywheel to a requested motor speed in RPS.
 *
 * <p>While scheduled, this command continuously calls
 * {@link frc.robot.subsystems.SK26Launcher#runLauncherRPS(double, String)} to maintain the target
 * motor speed. The command never finishes on its own; it is intended to be used with a
 * trigger/button binding.
 *
 * <p>When the command ends (button released or interrupted), the launcher is set to idle via
 * {@link frc.robot.subsystems.SK26Launcher#idleLauncher()} so it remains spun up for faster
 * subsequent shots.
 *
 * <p>Requires the {@link frc.robot.subsystems.SK26Launcher} subsystem.
 */
public class RunLauncherWithRPSCommand extends Command {

    SK26Launcher launchermotor;
    double targetMotorRPS;

    /**
     * Creates a command that runs the launcher to a target motor speed.
     *
     * @param launchermotor the launcher subsystem to control
     * @param targetMotorRPS desired motor speed in rotations per second (RPS)
     */
    public RunLauncherWithRPSCommand(SK26Launcher launchermotor, double targetMotorRPS) {
        this.launchermotor = launchermotor;
        this.targetMotorRPS = targetMotorRPS;
    }

    @Override
    public void initialize() {
        // no-op
    }

    @Override
    public void execute() {
        launchermotor.runLauncherRPS(targetMotorRPS, "Launching");
    }

    @Override
    public void end(boolean interrupted) {
        // launchermotor.stopLauncher();
        launchermotor.idleLauncher(); // makes the launcher quickly relaunchable
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
