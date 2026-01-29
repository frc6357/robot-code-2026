package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SK26Lights;

public class RequestSKBlueWave extends Command {

    private final SK26Lights Subsystem;

    // Constructor
    public RequestSKBlueWave(SK26Lights Subsystem){
        this.Subsystem = Subsystem;
        addRequirements(Subsystem);
    }

    @Override
    public void initialize(){
        Subsystem.setBreathingSKBlue();
    }

    @Override
    public boolean isFinished(){
        return true; // finish immediately after toggling
    }

    @Override
    public void end(boolean interrupted) {
    }
}