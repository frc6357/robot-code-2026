package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK26Turret;

public class StopLauncherCommand extends Command {
    private SK26Turret m_launcher;

    public StopLauncherCommand(SK26Turret m_launcher) {
        this.m_launcher = m_launcher;
    }

    @Override
    public void initialize() {
        m_launcher.stopLauncher();
    }
    
}
