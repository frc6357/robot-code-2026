package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SK26Indexer;

import static frc.robot.Konstants.IndexerConstants.kIndexerFeedRPS;

// Command to feed fuel using the indexer subsystem
public class IndexerFeedCommand extends Command {

    private final SK26Indexer Subsystem;

    // Constructor
    public IndexerFeedCommand(SK26Indexer Subsystem){
        this.Subsystem = Subsystem;
        addRequirements(Subsystem);
    }

    // When the command is initialized, start feeding fuel
    @Override
    public void initialize(){
        Subsystem.feedFuel(kIndexerFeedRPS);
    }

    // This command never finishes on its own
    @Override
    public boolean isFinished(){
        return false;
    }

    // When the command ends or is interrupted, set the indexer to idle
    @Override
    public void end(boolean interrupted) {
        Subsystem.idleIndexer();
    }
}