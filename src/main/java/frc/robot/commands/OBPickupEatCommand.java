package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pickupOB.SK26PickupOB;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OBPickupEatCommand extends Command {
  public final SK26PickupOB intakeSubsystem;
  /** Creates a new OBPickupEatCommand. */
  public OBPickupEatCommand(SK26PickupOB intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.runEaterMotor(); //defined in subsystem
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopEaterMotor(); //defined in subsystem
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
