package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK26Lights;

public class RequestSKBlueWave extends Command {

    private final SK26Lights subsystem;

    public RequestSKBlueWave(SK26Lights subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setBreathingSKBlue().schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}