package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.SK26Intake;

// Command to feed fuel using the intake subsystem
public class IntakeCommand extends Command {

    private final SK26Intake intake;
    double voltage;

    // Constructor
    public IntakeCommand(SK26Intake intake, double voltage)
    {
        this.intake = intake;
        this.voltage = voltage;

        addRequirements(intake);
    }

    // When the command is initialized, start intaking fuel
    @Override
    public void initialize()
    {
        intake.setIntakeVoltage(voltage);
    }

    // This command never finishes on its own
    @Override
    public boolean isFinished()
    {
        return false;
    }

    // When the command ends or is interrupted, set the intake to idle
    @Override
    public void end(boolean interrupted) {}
}