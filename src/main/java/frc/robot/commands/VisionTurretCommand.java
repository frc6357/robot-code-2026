package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK26Turret;
import frc.robot.subsystems.vision.SKVision;

public class VisionTurretCommand extends Command {

    private final SK26Turret turret;
    private final SKVision vision;
    private final double kP;

    /**
     * Vision turret command
     * @param turret the SK26Turret subsystem
     * @param vision the SKVision subsystem
     * @param kP proportional gain for error correction (tune as needed)
     */
    public VisionTurretCommand(SK26Turret turret, SKVision vision, double kP) 
    {
        this.turret = turret;
        this.vision = vision;
        this.kP = kP;

        addRequirements(turret);
    }

    @Override
    public void initialize() 
    {
        // Vision tracks to the limelight that can see the AprilTag the best
        vision.getBestLimelight().setLEDMode(true);
    }

    @Override
    public void execute() 
    {
        var limelight = vision.getBestLimelight();

        // If the imelight can see its target, the turret sets its rotation based on limelight feedback
        if (limelight != null && limelight.targetInView()) 
        {
            double yawError = limelight.getHorizontalOffset();
            double currentAngle = turret.getAngleDegrees();
            double targetAngle = currentAngle + yawError * kP;
            turret.setRotation2d(Rotation2d.fromDegrees(targetAngle));
            SK26Turret.lastTargetAngle = targetAngle;
        } 
        
        // If it cant see the limelight, hold the position instead
        else 
        {
            turret.holdPosition();
        }
    }


    @Override
    public void end(boolean interrupted) 
    {
        vision.getBestLimelight().setLEDMode(false);
        turret.holdPosition();
    }

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
