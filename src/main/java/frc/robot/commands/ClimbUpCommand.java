package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

import static frc.robot.Konstants.ClimbConstants.kClimbMotorSpeed;

public class ClimbUpCommand extends Command{

    private final Climb climb;

    public ClimbUpCommand(Climb climb)
    {
        this.climb = climb;

        addRequirements(climb);
    }

    @Override
    public void initialize()
    {}

    @Override
    public void execute()
    {
        climb.runMotors(kClimbMotorSpeed);
        climb.setIsRunning(true);
    }

    @Override
    public void end(boolean interrupted)
    {
        climb.stopMotors();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}

