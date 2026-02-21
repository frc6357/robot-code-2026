package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.SK26Intake;

import static frc.robot.Konstants.pickupOBConstants.kIntakeIdleSpeed;
import static frc.robot.Konstants.pickupOBConstants.kIntakeSpeed;

// Command to feed fuel using the indexer subsystem
public class IntakeCommand extends Command {

    private final SK26Intake intake;

    // Constructor
    public IntakeCommand(SK26Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    // When the command is initialized, start feeding fuel
    @Override
    public void initialize(){
        intake.runIntakeMotor(kIntakeSpeed);
    }

    // This command never finishes on its own
    @Override
    public boolean isFinished(){
        return false;
    }

    // When the command ends or is interrupted, set the indexer to idle
    @Override
    public void end(boolean interrupted) {
        intake.runIntakeMotor(kIntakeIdleSpeed);
    }
}