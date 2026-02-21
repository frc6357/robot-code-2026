package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.SK26Indexer;

// Command to feed fuel using the indexer subsystem
public class IndexerFeedCommand extends Command 
{

    private final SK26Indexer Subsystem;

    double velocity;

    // Constructor
    public IndexerFeedCommand(SK26Indexer Subsystem, double velocity)
    {
        this.Subsystem = Subsystem;
        this.velocity = velocity;
        addRequirements(Subsystem);
    }

    // When the command is initialized, start feeding fuel
    @Override
    public void initialize()
    {
        Subsystem.feedFuel(velocity);
    }

    // This command never finishes on its own
    @Override
    public boolean isFinished()
    {
        return false;
    }

    // When the command ends or is interrupted, set the indexer to idle
    @Override
    public void end(boolean interrupted) {}
}