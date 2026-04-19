package frc.robot.commands.automatedDriving;

import static frc.robot.Konstants.ClimbConstants.kApproachDistanceMeters;
import static frc.robot.Konstants.ClimbConstants.kClimbApproachConstraints;
import static frc.robot.Konstants.ClimbConstants.kTowerLeftRungBlue;
import static frc.robot.Konstants.ClimbConstants.kTowerRightRungBlue;
import static frc.robot.Konstants.ClimbConstants.kTowerLeftRungRed;
import static frc.robot.Konstants.ClimbConstants.kTowerRightRungRed;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.auto.Pathfinder;
import frc.lib.utils.Field;
import frc.robot.subsystems.drive.SKSwerve;

/**
 * Automated T1 climbing approach and alignment command.
 * 
 * <p>This command executes when the driver requests the CLIMBING state:
 * <ol>
 *   <li>Determines which tower post (left or right) is closest based on robot Y position</li>
 *   <li>Pathfinds to an approach pose 1.65m from the tower front face</li>
 *   <li>Uses a ProfiledPIDController to precisely align in X for the final "hug"</li>
 * </ol>
 * 
 * <p>The rotation remains static after pathfinding (facing driver-forward).
 */
public class ClimbApproachAndAlign {

    private ClimbApproachAndAlign() {
        // Utility class - no instantiation
    }

    /**
     * Creates the full climb approach and alignment command sequence.
     * 
     * @param drive The swerve drive subsystem
     * @return A command that pathfinds to the approach pose, then aligns in X
     */
    public static Command create(SKSwerve drive) {
        return Commands.sequence(
            // Phase 1: Pathfind to approach pose
            Commands.defer(() -> createPathfindCommand(drive), Set.of(drive)),
            // Phase 2: Final X alignment with ProfiledPIDController
            Commands.defer(() -> createAlignmentCommand(drive), Set.of(drive))
        ).withName("ClimbApproachAndAlign");
    }

    /**
     * Determines the target tower post based on which one the robot is closer to.
     * 
     * @param robotY Current robot Y position (meters)
     * @return The Translation2d of the closest tower upright (left or right)
     */
    private static Translation2d getClosestTowerUpright(double robotY) {
        Translation2d leftUpright;
        Translation2d rightUpright;

        if (Field.isBlue()) {
            leftUpright = kTowerLeftRungBlue;
            rightUpright = kTowerRightRungBlue;
        } else {
            leftUpright = kTowerLeftRungRed;
            rightUpright = kTowerRightRungRed;
        }

        double distToLeft = Math.abs(robotY - leftUpright.getY());
        double distToRight = Math.abs(robotY - rightUpright.getY());

        return distToLeft < distToRight ? leftUpright : rightUpright;
    }

    /**
     * Calculates the approach pose for the robot based on the target tower upright.
     * 
     * <p>The approach pose is positioned {@value frc.robot.Konstants.ClimbConstants#kApproachDistanceMeters}m
     * from the tower front face, with the robot facing driver-forward (0° on blue, 180° on red).
     * 
     * @param targetUpright The tower upright Translation2d
     * @return The approach Pose2d
     */
    private static Pose2d calculateApproachPose(Translation2d targetRung) {
        double approachX;
        Rotation2d approachRotation;

        if (Field.isBlue()) {
            // Blue alliance: tower is near X=0, approach from +X direction
            approachX = targetRung.getX() + kApproachDistanceMeters;
            approachRotation = Rotation2d.fromDegrees(0); // Facing driver-forward (+X)
        } else {
            // Red alliance: tower is near X=fieldLength, approach from -X direction
            approachX = targetRung.getX() - kApproachDistanceMeters;
            approachRotation = Rotation2d.fromDegrees(180); // Facing driver-forward (-X in WPILib coords)
        }

        return new Pose2d(approachX, targetRung.getY(), approachRotation);
    }

    /**
     * Creates the pathfinding command to the approach pose.
     * This is deferred to capture the robot's current position at execution time.
     */
    public static Command createPathfindCommand(SKSwerve drive) {
        double robotY = drive.getRobotPose().getY();
        Translation2d targetUpright = getClosestTowerUpright(robotY);
        Pose2d approachPose = calculateApproachPose(targetUpright);

        Logger.recordOutput("ClimbApproach/TargetUpright", targetUpright);
        Logger.recordOutput("ClimbApproach/ApproachPose", approachPose);

        return Pathfinder.PathfindToPoseCommand(approachPose, kClimbApproachConstraints, 0.0)
            .withName("ClimbPathfindToApproach");
    }

    /**
     * Creates the final X, Y, and rotational alignment command using ProfiledPIDControllers.
     * 
     * <p>This command drives the robot to align precisely with the tower,
     * using motion profiling for smooth acceleration. The command finishes when within
     * {@value frc.robot.Konstants.ClimbConstants#kAlignmentToleranceMeters}m of the target X,
     * {@value frc.robot.Konstants.ClimbConstants#kAlignmentYToleranceMeters}m of the target Y,
     * and {@value frc.robot.Konstants.ClimbConstants#kAlignmentRotToleranceRadians} radians of the target rotation.
     */
    public static Command createAlignmentCommand(SKSwerve drive) {
        return new ClimbAlignment(drive);
    }
}
