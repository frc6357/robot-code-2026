package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK26Lights;

public class RequestLEDWhite extends Command {
    private final SK26Lights lights;

    public RequestLEDWhite(SK26Lights lights) {
        this.lights = lights;
        addRequirements(lights);
    }

    @Override
    public void initialize() {
        lights.requestLEDWhite();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}