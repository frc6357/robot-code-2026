package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SK26Indexer;

import static frc.robot.Konstants.IndexerConstants.kIndexerFeedRPS;

public class IndexerFeedCommand extends Command {

    private final SK26Indexer Subsystem;

    public IndexerFeedCommand(SK26Indexer Subsystem){
        this.Subsystem = Subsystem;
    }

    @Override
    public void initialize(){
        Subsystem.feedFuel(kIndexerFeedRPS);
        SmartDashboard.putBoolean("IndexFeedCommandRunning", true);
    }
    
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("IndexFeedCommandRunning", false);
        Subsystem.feedFuel(0);
    }
}
