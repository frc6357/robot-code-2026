package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SK26Lights;

public class SKBlueWaveCommand extends Command {

    private final SK26Lights Subsystem;

    // Constructor
    public SKBlueWaveCommand(SK26Lights Subsystem){
        this.Subsystem = Subsystem;
        addRequirements(Subsystem);
    }

    @Override
    public void initialize(){
        if (Subsystem.skBlueWaveEnabled) {
            Subsystem.disableSKBlueWave();
        } else {
            Subsystem.enableSKBlueWave();
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}