package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Konstants.ClimbConstants.kClimbMotorSpeed;

import frc.robot.subsystems.Climb;

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
        climb.setClimbHeight(kClimbMotorSpeed);
        climb.isRunning = true;
    }

    @Override
    public void end(boolean interrupted)
    {
        climb.stopMotor();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}

