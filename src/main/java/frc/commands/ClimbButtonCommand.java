package frc.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Climb;

public class ClimbButtonCommand extends Command{

    private final Climb subsystem;
    private final double height;

    public ClimbButtonCommand(double height, Climb Subsystem)
    {
        this.subsystem = Subsystem;
        this.height = height;

        addRequirements(subsystem);
    }

    public void initialize()
    {
        subsystem.setClimbHeight(height);
    }

    public boolean isFinished()
    {
        if(DriverStation.isAutonomousEnabled())
        {
            if (subsystem.climbIsAtHeight())
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
            if (subsystem.climbIsAtHeight())
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
