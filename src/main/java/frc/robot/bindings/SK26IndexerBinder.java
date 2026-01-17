package frc.robot.bindings;

import frc.robot.subsystems.SK26Indexer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Ports.IndexerPorts;
import frc.robot.commands.IndexerFeedCommand;

import java.util.Optional;

import static frc.robot.Ports.OperatorPorts.kIndex;

public class SK26IndexerBinder implements CommandBinder{


    Optional<SK26Indexer> indexerSubsystem;

    Trigger Index;

    public SK26IndexerBinder(Optional<SK26Indexer> indexerSubsystem){
        this.indexerSubsystem = indexerSubsystem;
        this.Index = kIndex.button;
    }

    public void bindButtons(){
        if (indexerSubsystem.isPresent()){
            SK26Indexer indexer = indexerSubsystem.get();

            Index.whileTrue(new IndexerFeedCommand(indexer));
        }
    }
}
