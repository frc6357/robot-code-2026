package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK26Launcher;

public class RunLauncherWithRPSCommand extends Command {

    SK26Launcher launchermotor;
    double targetMotorRPS;
    
    public RunLauncherWithRPSCommand(SK26Launcher launchermotor, double targetMotorRPS) {
        this.launchermotor = launchermotor;
        this.targetMotorRPS = targetMotorRPS;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        launchermotor.runLauncherRPS(targetMotorRPS, "Launching");
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
