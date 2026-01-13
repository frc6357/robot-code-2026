package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK26Turret;

public class RunLauncherCommand extends Command {
    private SK26Turret m_launcher;
    private double speed;
    
    public RunLauncherCommand(SK26Turret m_launcher, double speed) {
        this.m_launcher = m_launcher;
        this.speed = speed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_launcher.runLauncher(speed);
    }
}
