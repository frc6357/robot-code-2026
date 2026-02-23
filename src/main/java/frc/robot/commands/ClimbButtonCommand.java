package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Climb;

public class ClimbButtonCommand extends Command{

    private final Climb climb;
    private final double height;

    public ClimbButtonCommand(double height, Climb climb)
    {
        this.climb = climb;
        this.height = height;

        addRequirements(climb);
    }

    public void initialize()
    {
        climb.setClimbHeight(height);
        climb.isRunning = true;
    }

    public boolean isFinished()
    {
        if(DriverStation.isAutonomousEnabled())
        {
            if (climb.climbIsAtHeight())
            {
                return true;
            }

            else
            {
                return false;
            }
        }
        else
        {
            if (climb.climbIsAtHeight())
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }
}
