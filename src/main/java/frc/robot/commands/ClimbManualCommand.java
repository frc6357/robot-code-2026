package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

import java.util.function.Supplier;

import static frc.robot.Konstants.ClimbConstants.kClimbMotorSpeed;
import static frc.robot.Konstants.ClimbConstants.kClimbTolerance;

public class ClimbManualCommand extends Command{

    private final Climb climb;
    private final Supplier<Double> padInput;

    public ClimbManualCommand(Supplier<Double> direction, Climb climb)
    {
        this.climb = climb;
        this.padInput = direction;

        addRequirements(climb);
    }

    @Override
    public void initialize(){}

    @Override 
    public void execute()
    {
        if (padInput.get() + kClimbTolerance == 270 || padInput.get() - kClimbTolerance == 270)
        {
            climb.runMotor(kClimbMotorSpeed);
            climb.isRunning = true;
        }
        else if (padInput.get() + kClimbTolerance == 90 || padInput.get() - kClimbTolerance == 90)
        {
            climb.runMotor(-kClimbMotorSpeed);
            climb.isRunning = true;
        }
        else 
        {
            climb.hold();
            if(climb.cAtMaxHeight())
            {
                climb.stopMotor();
                climb.setClimbHeight(climb.getClimbPosition()-1);
            }
        }
    }
}
