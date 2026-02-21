package frc.robot.bindings;

// Imports from robot
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.commands.IndexerFeedCommand;
import frc.robot.subsystems.indexer.SK26Indexer;
import static frc.robot.Konstants.IndexerConstants.kIndexerFullSpeed;
import static frc.robot.Konstants.IndexerConstants.kIndexerIdleSpeed;

// Imports from Java/WPILib
import java.util.Optional;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SK26IndexerBinder implements CommandBinder
{
    Optional<SK26Indexer> indexerSubsystem;

    Trigger IndexFeed;
    Trigger IndexIdle;
    Trigger IsIdle;

    public SK26IndexerBinder(Optional<SK26Indexer> indexerSubsystem)
    {
        this.indexerSubsystem = indexerSubsystem;

        // Feed when any shooting state is READY (launcher up to speed)
        IndexFeed = StateHandler.whenCurrentStateReady(MacroState.SCORING)
            .or(StateHandler.whenCurrentStateReady(MacroState.SHUTTLING))
            .or(StateHandler.whenCurrentStateReady(MacroState.STEADY_STREAM_SCORING))
            .or(StateHandler.whenCurrentStateReady(MacroState.STEADY_STREAM_SHUTTLING));

        // For simple trigger bindings (if necessary)
        IsIdle = StateHandler.whenCurrentState(MacroState.IDLE);
    }

    public void bindButtons()
    {
        if (indexerSubsystem.isEmpty())
        {
            return;
        }

        SK26Indexer indexer = indexerSubsystem.get();

        IndexFeed.whileTrue(new IndexerFeedCommand(indexer, kIndexerFullSpeed));
        IndexIdle.whileTrue(new IndexerFeedCommand(indexer, kIndexerIdleSpeed));
    }
}