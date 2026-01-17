package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK26Turret;
import frc.robot.subsystems.vision.SKVision;

public class VisionTurretCommand extends Command {

    private final SK26Turret turret;
    private final SKVision vision;
    private final double kP; // simple proportional control for motion magic adjustment

    /**
     * Vision-based turret command.
     * @param turret the SK26Turret subsystem
     * @param vision the SKVision subsystem (manages limelights)
     * @param kP proportional gain for error correction (tune as needed)
     */
    public VisionTurretCommand(SK26Turret turret, SKVision vision, double kP) {
        this.turret = turret;
        this.vision = vision;
        this.kP = kP;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        // Optional: ensure LEDs are on when starting vision tracking
        vision.getBestLimelight().setLEDMode(true);
    }

    @Override
    public void execute() {
        var limelight = vision.getBestLimelight();

        if (limelight != null && limelight.targetInView()) 
        {
            double yawError = limelight.getHorizontalOffset();
            double currentAngle = turret.getAngleDegrees();
            double targetAngle = currentAngle + yawError * kP;
            turret.setRotation2d(Rotation2d.fromDegrees(targetAngle));
            SK26Turret.lastTargetAngle = targetAngle;
        } 
        else 
        {
            turret.holdPosition();
        }
    }


    @Override
    public void end(boolean interrupted) {
        // Optional: turn off LEDs when stopping vision tracking
        vision.getBestLimelight().setLEDMode(false);

        // Hold the last position when command ends
        turret.holdPosition();
    }

    @Override
    public boolean isFinished() {
        return false; // never finishes on its own
    }
}
