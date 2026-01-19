
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pickupOB.SKpickupOB;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OBPickupPositionCommand extends Command {
  SKpickupOB position;
  /** Creates a new OBPickupPositionCommand. */
  public OBPickupPositionCommand(SKpickupOB position) {
    this.position = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(position);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    position.runPositionerMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    position.stopPositionerMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
