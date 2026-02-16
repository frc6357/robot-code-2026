package frc.robot.commands.commandGroups;

import static frc.robot.Konstants.IndexerConstants.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SK26Indexer;

public class UnjamCommand extends SequentialCommandGroup {
  public UnjamCommand(SK26Indexer indexer, SK26Indexer spindexer ) {

    addRequirements(indexer);

    addCommands(
        Commands.sequence(
                // Reverse
                Commands.runOnce(() -> indexer.unjamIndexer(kIndexerUnjamReverseRPS), indexer),
                Commands.runOnce(() -> spindexer.unjamIndexer(kIndexerUnjamReverseRPS), indexer),
                Commands.waitSeconds(kIndexerUnjamReverseDuration),

                // Pause
                Commands.runOnce(() -> indexer.unjamIndexer(kIndexerIdleRPS), indexer),
                Commands.runOnce(() -> spindexer.unjamIndexer(kIndexerIdleRPS), indexer),
                Commands.waitSeconds(kIndexerUnjamWaitDuration),

                // Forward
                Commands.runOnce(() -> indexer.unjamIndexer(kIndexerUnjamForwardRPS), indexer),
                Commands.runOnce(() -> spindexer.unjamIndexer(kIndexerUnjamForwardRPS), indexer),
                Commands.waitSeconds(kIndexerUnjamForwardDuration),

                // Pause
                Commands.runOnce(() -> indexer.unjamIndexer(kIndexerIdleRPS), indexer),
                Commands.runOnce(() -> spindexer.unjamIndexer(kIndexerIdleRPS), indexer),
                Commands.waitSeconds(kIndexerUnjamWaitDuration))

            // Repeat until interrupted
            .repeatedly()

            // When interrupted, set indexer to idle
            .finallyDo(() -> indexer.idleIndexer()));
            
  }
}