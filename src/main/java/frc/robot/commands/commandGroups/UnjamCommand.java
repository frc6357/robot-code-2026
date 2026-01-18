package frc.robot.commands.commandGroups;

import static frc.robot.Konstants.IndexerConstants.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SK26Indexer;

public class UnjamCommand extends SequentialCommandGroup {
  public UnjamCommand(SK26Indexer indexer) {
    addRequirements(indexer);

    addCommands(
        Commands.sequence(
                // Reverse
                Commands.runOnce(() -> indexer.setIndexerVelocity(kIndexerUnjamReverseRPS), indexer),
                Commands.waitSeconds(kIndexerUnjamReverseDuration),

                // Pause
                Commands.runOnce(() -> indexer.setIndexerVelocity(kIndexerIdleRPS), indexer),
                Commands.waitSeconds(kIndexerUnjamWaitDuration),

                // Forward
                Commands.runOnce(() -> indexer.setIndexerVelocity(kIndexerUnjamForwardRPS), indexer),
                Commands.waitSeconds(kIndexerUnjamForwardDuration),

                // Pause
                Commands.runOnce(() -> indexer.setIndexerVelocity(kIndexerIdleRPS), indexer),
                Commands.waitSeconds(kIndexerUnjamWaitDuration))
            .repeatedly()
            .finallyDo(() -> indexer.setIndexerVelocity(kIndexerIdleRPS)));
  }
}