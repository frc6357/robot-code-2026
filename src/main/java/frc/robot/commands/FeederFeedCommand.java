package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.SK26Feeder;

// Command to feed fuel using the feeder subsystem
public class FeederFeedCommand extends Command 
{

    private final SK26Feeder Subsystem;

    double velocity;

    // Constructor
    public FeederFeedCommand(SK26Feeder Subsystem, double velocity)
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

    @Override
    public void end(boolean interrupted) {}
}