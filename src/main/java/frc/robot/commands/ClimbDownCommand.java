package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

import static frc.robot.Konstants.ClimbConstants.kClimbMotorSpeed;

public class ClimbDownCommand extends Command{

    private final Climb climb;

    public ClimbDownCommand(Climb climb)
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
        SmartDashboard.putNumber("speed", kClimbMotorSpeed);
        climb.runMotors(-kClimbMotorSpeed);
        climb.isRunning = true;
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


