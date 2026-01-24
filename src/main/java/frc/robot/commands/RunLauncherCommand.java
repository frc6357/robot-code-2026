package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK26Launcher;

public class RunLauncherCommand extends Command {

    SK26Launcher launchermotor;
    double targetLaunchVelocity;
    
    public RunLauncherCommand(SK26Launcher launchermotor, double targetLaunchVelocity) {
        this.launchermotor = launchermotor;
        this.targetLaunchVelocity = targetLaunchVelocity;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        launchermotor.runLauncherExitVel(targetLaunchVelocity, "Launching");
    }

    @Override
    public void end(boolean interrupted) {
        launchermotor.stopLauncher();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
