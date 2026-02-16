package frc.robot.commands;

import static frc.robot.Konstants.pickupOBConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pickupOB.SK26PickupOB;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OBPickupSpitCommand extends Command {
  public final SK26PickupOB spit;
  /** Creates a new OBPickupSpitCommand. */
  public OBPickupSpitCommand(SK26PickupOB spit) {
    this.spit = spit;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spit);
  }

  // Called when th e command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spit.runEaterMotorReverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spit.stopEaterMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
