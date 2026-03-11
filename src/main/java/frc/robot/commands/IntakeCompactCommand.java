package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.SK26Intake;

/**
 * Command that oscillates the intake up and down to gently spill balls 
 * from the hopper into the indexer. The intake pivots between a high 
 * and low position in a continuous rocking motion.
 */
public class IntakeCompactCommand extends Command {

    private final SK26Intake intake;
    
    // Oscillation positions (in rotations)
    private final double stowPosition;
    private final double groundPosition;
    
    // Track which direction we're moving
    private boolean movingToHigh = true;

    /**
     * Creates a new IntakeCompactCommand with custom oscillation range.
     * @param intake The intake subsystem
     * @param groundPosition The low position in rotations
     * @param stowPosition The high position in rotations
     */
    public IntakeCompactCommand(SK26Intake intake, double groundPosition, double stowPosition) {
        this.intake = intake;
        this.groundPosition = groundPosition;
        this.stowPosition = stowPosition;

        addRequirements(intake);
    }

    @Override
    public void initialize() 
    {
        // Start by moving to high position
        movingToHigh = true;
        intake.setTargetPosition(stowPosition);
    }

    @Override
    public void execute() {
        if (intake.isPositionerAtTarget()) 
        {
            if (movingToHigh) {
                // At high position, go low
                movingToHigh = false;
                intake.setTargetPosition(groundPosition);
            } 
            else {
                // At low position, go high
                movingToHigh = true;
                intake.setTargetPosition(stowPosition);
            }
        }
    }

    @Override
    public void end(boolean interrupted) 
    {
        intake.setTargetPosition(intake.getCurrentPosition());
    }

    @Override
    public boolean isFinished() 
    {
        // This command runs continuously until interrupted
        return false;
    }
}