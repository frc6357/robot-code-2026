package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.SK26Climb;

import static frc.robot.Konstants.ClimbConstants.kClimbMotorSpeed;

public class ClimbDownCommand extends Command{

    private final SK26Climb climb;

    public ClimbDownCommand(SK26Climb climb)
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
        climb.runMotors(-kClimbMotorSpeed);
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


