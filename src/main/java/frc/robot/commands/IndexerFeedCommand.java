package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SK26Indexer;

import static frc.robot.Konstants.IndexerConstants.kIndexerMotorSpeedRPS;

public class IndexerFeedCommand extends Command {

    private final SK26Indexer Subsystem;

    public IndexerFeedCommand(SK26Indexer Subsystem){
        this.Subsystem = Subsystem;
    }

    public void initialize(){
        Subsystem.feedFuel(kIndexerMotorSpeedRPS);
    }
    
    public boolean isFinished(){
        return true;
    }
}
