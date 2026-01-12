package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SKLauncher;

public class RunLauncherCommand extends Command {
    private SKLauncher m_launcher;
    private double speed;
    
    public RunLauncherCommand(SKLauncher m_launcher, double speed) {
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
