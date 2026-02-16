package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pickupOB.SK26PickupOB;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OBPickupRetractCommand extends Command {
  public final SK26PickupOB retract;
  /** Creates a new OBPickupRetractCommand. */
  public OBPickupRetractCommand(SK26PickupOB retract) {
    this.retract = retract;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(retract);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    retract.runPositionerMotorReverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    retract.stopPositionerMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
