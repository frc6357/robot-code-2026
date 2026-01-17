package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK26Launcher;

public class RunLauncherCommand extends Command {

    SK26Launcher launchermotor;
    
    public RunLauncherCommand(SK26Launcher launchermotor) {
        this.launchermotor = launchermotor;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        launchermotor.startLauncher(1);
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
