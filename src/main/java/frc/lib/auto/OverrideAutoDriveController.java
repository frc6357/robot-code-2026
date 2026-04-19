package frc.lib.auto;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.Command;

/** 
 * Part of the unfinished work related to {@link OverrideAuto}. </p>
 * This command is intended to override the translation and rotation controllers of the Pathplanner driving system, 
 * allowing us to interject a command into an already running PathPlannerAuto command, then after that command finishes, 
 * have the robot pathfind to the next pose in the currently active path to "recover" 
 * and continue on as if the auto had not been interrupted. 
 */
public class OverrideAutoDriveController extends Command {
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotSupplier;
    

    /**
     * Creates a command that overrides the translation and rotation controllers of the
     * Pathplanner driving system. Use this command with a race condition, deadline, etc. since
     * it never ends on its own.
     * @param xSupplier The supplier for the x velocity feedback
     * @param ySupplier The supplier for the y velocity feedback
     * @param rotSupplier The supplier for the rotation velocity feedback
     */
    public OverrideAutoDriveController(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
    }

    @Override
    public void initialize() {
        PPHolonomicDriveController.overrideXYFeedback(xSupplier, ySupplier);
        PPHolonomicDriveController.overrideRotationFeedback(rotSupplier);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        PPHolonomicDriveController.clearXYFeedbackOverride();
        PPHolonomicDriveController.clearRotationFeedbackOverride();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}