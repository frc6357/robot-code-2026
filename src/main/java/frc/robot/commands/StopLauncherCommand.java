package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SKLauncher;

public class StopLauncherCommand extends Command {
    private SKLauncher m_launcher;

    public StopLauncherCommand(SKLauncher m_launcher) {
        this.m_launcher = m_launcher;
    }

    @Override
    public void initialize() {
        m_launcher.stopLauncher();
    }
    
}
