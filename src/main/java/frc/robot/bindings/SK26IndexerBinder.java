package frc.robot.bindings;

import frc.robot.subsystems.SK26Indexer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Ports.IndexerPorts;

import frc.robot.commands.IndexerFeedCommand;
import static frc.robot.Konstants.IndexerConstants.*;

import java.util.Optional;

import static frc.robot.Ports.OperatorPorts.kIndexFeed;
import static frc.robot.Ports.OperatorPorts.kIndexUnjam;

public class SK26IndexerBinder implements CommandBinder{


    Optional<SK26Indexer> indexerSubsystem;

    Trigger IndexFeed;
    Trigger IndexUnjam;

    public SK26IndexerBinder(Optional<SK26Indexer> indexerSubsystem){
        this.indexerSubsystem = indexerSubsystem;
        this.IndexFeed = kIndexFeed.button;
        this.IndexUnjam = kIndexUnjam.button;
    }

    public void bindButtons(){
        if (indexerSubsystem.isPresent()){
            SK26Indexer indexer = indexerSubsystem.get();

            IndexFeed.whileTrue(new IndexerFeedCommand(indexer));
            // IndexUnjam.whileTrue(
            //     Commands.sequence(
            //         indexer.setIndexerVelCommand(kIndexerUnjamReverseRPS),
            //         new WaitCommand(kIndexerUnjamReverseDuration),

            //         indexer.setIndexerVelCommand(kIndexerIdleRPS),
            //         new WaitCommand(kIndexerUnjamWaitDuration),

            //         indexer.setIndexerVelCommand(kIndexerUnjamForwardRPS),
            //         new WaitCommand(kIndexerUnjamForwardDuration),

            //         indexer.setIndexerVelCommand(kIndexerIdleRPS),
            //         new WaitCommand(kIndexerUnjamWaitDuration)
            // ).handleInterrupt(() -> indexer.setIndexerVelCommand(kIndexerIdleRPS)));
        }
    }
}
