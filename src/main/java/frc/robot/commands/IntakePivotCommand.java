package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.SK26Intake;

import frc.robot.Konstants.IntakeConstants.IntakePosition;

public class IntakePivotCommand extends Command{

    private final SK26Intake intake;
    private final IntakePosition angle;


    public IntakePivotCommand(SK26Intake intake, IntakePosition angle)
    {
        this.angle = angle;
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize()
    {
        intake.setPositionerPosition(angle);
    }

    @Override
    public boolean isFinished()
    {
        return intake.isPositionerAtTarget();
    }
}