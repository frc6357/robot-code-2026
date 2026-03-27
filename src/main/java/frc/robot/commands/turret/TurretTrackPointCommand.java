package frc.robot.commands.turret;

import static frc.robot.Konstants.LauncherConstants.kRobotToShooter;
import static frc.robot.Konstants.TurretConstants.kTurretCoordinateOffset;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.drive.SKTargetPoint;
import frc.robot.subsystems.turret.SK26Turret;

/**
 * Command that makes the turret continuously track a target point in field space.
 * The turret will automatically adjust its angle to keep pointing at the target
 * regardless of where the robot moves on the field.
 */
public class TurretTrackPointCommand extends Command
{
    private final SK26Turret turret;
    private final SKSwerve drive;
    private Optional<Translation2d> targetTranslation = Optional.empty();
    private Optional<SKTargetPoint> targetPoint = Optional.empty();

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
        this.targetPoint = Optional.of(targetPoint);
    
        addRequirements(turret);
    }

    public TurretTrackPointCommand(SK26Turret turret, SKSwerve drive, Translation2d targetTranslation) {
        this.turret = turret;
        this.drive = drive;
        this.targetTranslation = Optional.of(targetTranslation);

        addRequirements(turret);
    }

    @Override
    public void initialize()
    {
        // Sync to current turret angle to prevent any initial jump
        turret.setAngleDegrees(turret.getCachedAngleDegrees());
    }

    @Override
    public void execute()
    {
        Translation2d target;
        if(targetPoint.isPresent()) {
            target = targetPoint.get().getTargetPoint();
        }
        else {
            target = targetTranslation.get();
        }

        // Get the robot's current position and rotation
        Translation2d shooterPosition = drive.getRobotPose().getTranslation().plus(kRobotToShooter.getTranslation().toTranslation2d());
        double robotHeadingDeg = drive.getRobotRotation().getDegrees();

        // Calculate the field-relative angle FROM the shooter TO the target (in degrees)
        // atan2(dy, dx) gives the angle from positive X-axis, counterclockwise positive
        double dx = target.getX() - shooterPosition.getX();
        double dy = target.getY() - shooterPosition.getY();
        double fieldAngleToTarget = Math.toDegrees(Math.atan2(dy, dx));
        
        if(Double.isNaN(fieldAngleToTarget)) {
            fieldAngleToTarget = robotHeadingDeg + turret.getCachedAngleDegrees();
        }

        // Convert field-relative angle to robot-relative angle for the turret
        // Standard robot-relative: 0° = front, +90° = left
        // Turret coordinates: 0° = left, +90° = front
        // turretAngle = (fieldAngle - robotHeading) - kTurretCoordinateOffset
        double turretAngleDeg = fieldAngleToTarget - robotHeadingDeg - kTurretCoordinateOffset;

        double wrappedAngle = MathUtil.inputModulus(turretAngleDeg, -180, 180);

        turret.setAngleDegrees(wrappedAngle);

        // Debug output
        Logger.recordOutput("TurretTrack/FieldAngleToTarget", fieldAngleToTarget);
        Logger.recordOutput("TurretTrack/WrappedDesiredAngle", wrappedAngle);
        Logger.recordOutput("TurretTrack/RobotHeading", robotHeadingDeg);
        Logger.recordOutput("TurretTrack/DesiredTurretAngle", turretAngleDeg);
        Logger.recordOutput("TurretTrack/DistanceToTarget", shooterPosition.getDistance(target));
    }

    @Override
    public void end(boolean interrupted)
    {
        // Keep turret at its current position when command ends
        turret.setAngleDegrees(turret.getCachedAngleDegrees());
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
