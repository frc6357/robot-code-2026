package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK26Lights;

public class TurnOnLights extends Command {

    SK26Lights lights;
    int[] rgb;

    public TurnOnLights(SK26Lights lights, int[] rgb) {
        this.lights = lights;
        this.rgb = rgb;
    }

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        // Code to turn on the lights
        lights.turnOnLights(rgb);
    }

    @Override
    public void end(boolean interrupted) {
        // Code to turn off the lights if needed
        lights.turnOffLights();
    }

    @Override
    public boolean isFinished() {
        return false; // Command completes immediately after turning on the lights
    }
    
}
