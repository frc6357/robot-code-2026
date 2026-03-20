package frc.robot.commands;

import static frc.robot.Konstants.IntakeConstants.kIntakeCompactSwitchIntervalSeconds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.SK26IntakePivot;

/**
 * Command that oscillates the intake up and down to gently spill balls 
 * from the hopper into the indexer. The intake pivots between a high 
 * and low position in a continuous rocking motion based on a timer.
 */
public class IntakeCompactCommand extends Command {

    private final SK26IntakePivot intake;
    
    // Oscillation positions (in rotations)
    private final double compactPosition;
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
     * @param compactPosition The high position in rotations
     */
    public IntakeCompactCommand(SK26IntakePivot intake, double groundPosition, double compactPosition) {
        this(intake, groundPosition, compactPosition, kIntakeCompactSwitchIntervalSeconds);
    }

    /**
     * Creates a new IntakeCompactCommand with custom oscillation range and interval.
     * @param intake The intake pivot subsystem
     * @param groundPosition The low position in rotations
     * @param compactPosition The compacting position in rotations
     * @param switchIntervalSeconds Time between position switches in seconds
     */
    public IntakeCompactCommand(SK26IntakePivot intake, double groundPosition, double compactPosition, double switchIntervalSeconds) {
        this.intake = intake;
        this.groundPosition = groundPosition;
        this.compactPosition = compactPosition;
        this.switchIntervalSeconds = switchIntervalSeconds;

        addRequirements(intake);
    }

    @Override
    public void initialize() 
    {
        // Start by moving to high position and record the start time
        movingToHigh = true;
        lastSwitchTimestamp = Timer.getFPGATimestamp();
        intake.setTargetPosition(compactPosition);
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
                intake.setTargetPosition(compactPosition);
            }
        }
    }

    @Override
    public void end(boolean interrupted) 
    {
        intake.setTargetPosition(groundPosition); // Ensure intake is down when command ends
    }

    @Override
    public boolean isFinished() 
    {
        // This command runs continuously until interrupted
        return false;
    }
}