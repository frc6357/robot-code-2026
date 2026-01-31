package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK26Turret;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.drive.SKTargetPoint;

/**
 * Command that makes the turret continuously track a target point in field space.
 * The turret will automatically adjust its angle to keep pointing at the target
 * regardless of where the robot moves on the field.
 */
public class TurretTrackPointCommand extends Command
{
    private final SK26Turret turret;
    private final SKSwerve drive;
    private final SKTargetPoint targetPoint;

    /**
     * Creates a new TurretTrackPointCommand.
     * @param turret The turret subsystem to control
     * @param drive The swerve drive subsystem (used to get robot pose)
     * @param targetPoint The target point subsystem to track
     */
    public TurretTrackPointCommand(SK26Turret turret, SKSwerve drive, SKTargetPoint targetPoint)
    {
        this.turret = turret;
        this.drive = drive;
        this.targetPoint = targetPoint;

        addRequirements(turret);
    }

    @Override
    public void initialize()
    {
        // Sync to current turret angle to prevent any initial jump
        turret.setAngleDegrees(turret.getAngleDegrees());
    }

    @Override
    public void execute()
    {
        // Get the robot's current position and rotation
        Translation2d robotPosition = drive.getRobotPose().getTranslation();
        double robotHeadingDeg = drive.getRobotRotation().getDegrees();

        // Get the target point position
        Translation2d target = targetPoint.getTargetPoint();

        // Calculate the vector from robot to target
        double dx = target.getX() - robotPosition.getX();
        double dy = target.getY() - robotPosition.getY();

        // Calculate the field-relative angle to the target (in degrees)
        // atan2 gives angle from positive X-axis, counterclockwise positive
        double fieldAngleToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Convert field-relative angle to robot-relative angle for the turret
        // If robot is facing 0° and target is at 45° field-relative, turret should be at 45°
        // If robot is facing 30° and target is at 45° field-relative, turret should be at 15°
        double turretAngleDeg = fieldAngleToTargetDeg - robotHeadingDeg;

        // Normalize to -180 to +180 range
        while (turretAngleDeg > 180)
        {
            turretAngleDeg -= 360;
        }
        while (turretAngleDeg < -180)
        {
            turretAngleDeg += 360;
        }

        // Use wrapped angle setting to handle limits gracefully
        turret.setAngleDegreesWrapped(turretAngleDeg);

        // Debug output
        SmartDashboard.putNumber("TurretTrack/FieldAngleToTarget", fieldAngleToTargetDeg);
        SmartDashboard.putNumber("TurretTrack/RobotHeading", robotHeadingDeg);
        SmartDashboard.putNumber("TurretTrack/DesiredTurretAngle", turretAngleDeg);
        SmartDashboard.putNumber("TurretTrack/DistanceToTarget", robotPosition.getDistance(target));
    }

    @Override
    public void end(boolean interrupted)
    {
        // Keep turret at its current position when command ends
        turret.setAngleDegrees(turret.getAngleDegrees());
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
