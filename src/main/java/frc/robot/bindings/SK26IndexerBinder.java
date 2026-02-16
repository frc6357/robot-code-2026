package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kLBbutton;
import static frc.robot.Ports.OperatorPorts.kRBbutton;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IndexerFeedCommand;
import frc.robot.commands.commandGroups.UnjamCommand;
import frc.robot.subsystems.indexer.SK26Indexer;

public class SK26IndexerBinder implements CommandBinder{


    Optional<SK26Indexer> indexerSubsystem;

    Trigger IndexFeed;
    Trigger IndexUnjam;

    public SK26IndexerBinder(Optional<SK26Indexer> indexerSubsystem){
        this.indexerSubsystem = indexerSubsystem;
        this.IndexFeed = kRBbutton.button;
        this.IndexUnjam = kLBbutton.button;
    }

    public void bindButtons(){
        if (indexerSubsystem.isPresent()){
            SK26Indexer indexer = indexerSubsystem.get();
            SK26Indexer spindexer = indexerSubsystem.get();

            IndexFeed.whileTrue(new IndexerFeedCommand(indexer));
            IndexUnjam.whileTrue(new UnjamCommand(indexer, spindexer));
        }
    }
}
