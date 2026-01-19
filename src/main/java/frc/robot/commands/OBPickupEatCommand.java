package frc.robot.commands;

import static frc.robot.Konstants.pickupOBConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pickupOB.SKpickupOB;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OBPickupEatCommand extends Command {
  public final SKpickupOB eat;
  /** Creates a new OBPickupEatCommand. */
  public OBPickupEatCommand(SKpickupOB eat) {
    this.eat = eat;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(eat);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    eat.runEaterMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    eat.stopEaterMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
