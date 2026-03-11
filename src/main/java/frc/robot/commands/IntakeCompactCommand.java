package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.SK26Intake;

/**
 * Command that oscillates the intake up and down to gently spill balls 
 * from the hopper into the indexer. The intake pivots between a high 
 * and low position in a continuous rocking motion based on a timer.
 */
public class IntakeCompactCommand extends Command {

    private final SK26Intake intake;
    
    // Oscillation positions (in rotations)
    private final double stowPosition;
    private final double groundPosition;
    
    // Time between position switches (in seconds)
    private final double switchIntervalSeconds;
    
    // Track which direction we're moving
    private boolean movingToHigh = true;
    
    // Timer for position switching
    private double lastSwitchTimestamp;

    /**
     * Creates a new IntakeCompactCommand with custom oscillation range and default 1 second interval.
     * @param intake The intake subsystem
     * @param groundPosition The low position in rotations
     * @param stowPosition The high position in rotations
     */
    public IntakeCompactCommand(SK26Intake intake, double groundPosition, double stowPosition) {
        this(intake, groundPosition, stowPosition, 0.67);
    }

    /**
     * Creates a new IntakeCompactCommand with custom oscillation range and interval.
     * @param intake The intake subsystem
     * @param groundPosition The low position in rotations
     * @param stowPosition The high position in rotations
     * @param switchIntervalSeconds Time between position switches in seconds
     */
    public IntakeCompactCommand(SK26Intake intake, double groundPosition, double stowPosition, double switchIntervalSeconds) {
        this.intake = intake;
        this.groundPosition = groundPosition;
        this.stowPosition = stowPosition;
        this.switchIntervalSeconds = switchIntervalSeconds;

        addRequirements(intake);
    }

    @Override
    public void initialize() 
    {
        // Start by moving to high position and record the start time
        movingToHigh = true;
        lastSwitchTimestamp = Timer.getFPGATimestamp();
        intake.setTargetPosition(stowPosition);
    }

    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastSwitchTimestamp >= switchIntervalSeconds) 
        {
            lastSwitchTimestamp = currentTime;
            
            if (movingToHigh) 
            {
                // Switch to low position
                movingToHigh = false;
                intake.setTargetPosition(groundPosition);
                
            } 
            else 
            {
                // Switch to high position
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