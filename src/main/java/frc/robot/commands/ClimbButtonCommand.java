package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class ClimbButtonCommand extends Command{

    private final Climb climb;
    private final double height;

    public ClimbButtonCommand(double height, Climb climb)
    {
        this.climb = climb;
        this.height = height;

        addRequirements(climb);
    }

    @Override
    public void initialize()
    {
        climb.setClimbHeight(height);
        climb.setIsRunning(true);
    }

    @Override
    public boolean isFinished()
    {
        return climb.climbIsAtHeight();
    }
}
