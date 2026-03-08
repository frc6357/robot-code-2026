package frc.robot.commands.automatedDriving;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.auto.Pathfinder;
import frc.lib.utils.Field;
import frc.lib.utils.FieldConstants;
import frc.lib.utils.FieldConstants.LeftTrench;
import frc.lib.utils.FieldConstants.LinesVertical;
import frc.lib.utils.FieldConstants.RightTrench;

/**
 * Utility class that determines the optimal trench traversal path based on
 * the robot's current position and velocity.
 *
 * <p>Decision logic:
 * <ol>
 *   <li><b>Which trench?</b> — Determined by the robot's lateral (Y) position
 *       relative to the field centerline, biased by the robot's lateral velocity.
 *       A velocity bias term trades off a greater distance for a smaller velocity
 *       change, preventing the robot from choosing a trench it is actively moving
 *       away from.</li>
 *   <li><b>Which direction?</b> — Determined by the robot's longitudinal (X)
 *       position relative to the midpoint of the trench corridor. A robot that
 *       is on the alliance side enters from alliance; a robot on the neutral
 *       side enters from the neutral zone.</li>
 *   <li><b>Which path?</b> — One of the four pre-built PathPlanner paths:
 *       {@code LTrenchToAlliance}, {@code LTrenchToNeutral},
 *       {@code RTrenchToAlliance}, {@code RTrenchToNeutral}.</li>
 * </ol>
 *
 * <p>All position-based decisions are performed in blue-origin field coordinates
 * after flipping the robot's pose when on the red alliance, so the same
 * thresholds work for both alliances.
 */
public final class TravelUnderNearestTrench {

    // ────────────────────── Field geometry (blue-origin) ──────────────────────

    /**
     * Y-coordinate of the left trench corridor center (top of field, blue
     * perspective). Derived from the midpoint of the left trench opening.
     */
    private static final double LEFT_TRENCH_Y =
            LeftTrench.openingTopLeft.getY()
                    - LeftTrench.openingWidth / 2.0;

    /**
     * Y-coordinate of the right trench corridor center (bottom of field, blue
     * perspective). Derived from the midpoint of the right trench opening.
     */
    private static final double RIGHT_TRENCH_Y =
            RightTrench.openingTopRight.getY()
                    + RightTrench.openingWidth / 2.0;

    /**
     * X-coordinate of the alliance-side boundary of the trench corridor.
     * Uses the alliance-zone line as the starting reference.
     */
    private static final double TRENCH_ALLIANCE_X = LinesVertical.allianceZone;

    /**
     * X-coordinate of the neutral-side boundary of the trench corridor.
     * Uses the near neutral-zone line as the ending reference.
     */
    private static final double TRENCH_NEUTRAL_X = LinesVertical.neutralZoneNear;

    /**
     * X-midpoint of the trench corridor — the decision boundary between
     * entering from the alliance zone vs. the neutral zone.
     */
    private static final double TRENCH_MID_X =
            (TRENCH_ALLIANCE_X + TRENCH_NEUTRAL_X) / 2.0;

    // ──────────────────────── Tuning constants ────────────────────────────────

    /**
     * Weight (in meters) applied to the robot's lateral velocity (m/s) when
     * biasing the trench selection. Higher values make the robot prefer the
     * trench it is already moving toward, even if it is farther away.
     *
     * <p>A value of 1.0 means that 1 m/s of lateral velocity offsets the
     * effective Y-position by 1 meter.
     */
    private static final double VELOCITY_BIAS_WEIGHT = 1.0;

    private TravelUnderNearestTrench() {} // Static-only utility class

    // ──────────────────────────── Public API ──────────────────────────────────

    /**
     * Builds a command that pathfinds to—and follows—the optimal trench
     * traversal path for the robot's current state.
     *
     * <p>This method is designed to be called at command-schedule time (e.g.
     * inside a {@code Trigger.onTrue(() -> ...)}), so the decision is made with
     * the freshest pose/velocity at the instant the button is pressed.
     *
     * @param robotPose   The robot's current field-relative pose.
     * @param chassisSpeeds The robot's current field-relative chassis speeds.
     * @param constraints PathPlanner constraints for the pathfinding segment.
     * @return A command that pathfinds to the start of the chosen path, then
     *         follows it through the trench.
     */
    public static Command createCommand(
            Pose2d robotPose,
            ChassisSpeeds chassisSpeeds,
            PathConstraints constraints) {

        String pathName = choosePath(robotPose, chassisSpeeds);
        return Pathfinder.PathfindThenFollowPathCommand(pathName, constraints)
                .withName("TravelUnderTrench(" + pathName + ")");
    }

    // ──────────────────────── Decision helpers ────────────────────────────────

    /**
     * Chooses one of the four trench paths based on pose and velocity.
     *
     * @param robotPose   Field-relative pose (may be red or blue origin).
     * @param chassisSpeeds Field-relative speeds.
     * @return The PathPlanner path file name to follow.
     */
    static String choosePath(Pose2d robotPose, ChassisSpeeds chassisSpeeds) {
        // Normalize everything to blue-origin coordinates so a single set of
        // thresholds works for both alliances.
        Translation2d bluePos = toBlueOrigin(robotPose.getTranslation());
        ChassisSpeeds blueSpeeds = toBlueOriginSpeeds(chassisSpeeds);

        boolean useLeftTrench = shouldUseLeftTrench(bluePos, blueSpeeds);
        boolean enterFromAlliance = shouldEnterFromAlliance(bluePos);

        if (useLeftTrench) {
            return enterFromAlliance ? "LTrenchToNeutral" : "LTrenchToAlliance";
        } else {
            return enterFromAlliance ? "RTrenchToNeutral" : "RTrenchToAlliance";
        }
    }

    /**
     * Determines whether the left trench (high-Y, blue perspective) is the
     * better choice.
     *
     * <p>The robot's lateral velocity is used as a bias: if the robot is
     * moving strongly toward the right trench, it will prefer the right trench
     * even if it is slightly farther in pure distance.
     *
     * @param bluePos   Robot position in blue-origin coordinates.
     * @param blueSpeeds Robot velocity in blue-origin coordinates.
     * @return {@code true} to use the left trench, {@code false} for right.
     */
    private static boolean shouldUseLeftTrench(Translation2d bluePos, ChassisSpeeds blueSpeeds) {
        double distToLeft = Math.abs(bluePos.getY() - LEFT_TRENCH_Y);
        double distToRight = Math.abs(bluePos.getY() - RIGHT_TRENCH_Y);

        // Positive vy = moving toward higher Y = toward the left trench.
        // Bias: subtract cost from the trench we are moving toward.
        double vy = blueSpeeds.vyMetersPerSecond;
        double biasedDistToLeft = distToLeft - (vy * VELOCITY_BIAS_WEIGHT);
        double biasedDistToRight = distToRight + (vy * VELOCITY_BIAS_WEIGHT);

        return biasedDistToLeft <= biasedDistToRight;
    }

    /**
     * Determines whether the robot should enter the trench from the alliance
     * side (and therefore travel toward the neutral zone) or from the neutral
     * side (traveling toward the alliance zone).
     *
     * @param bluePos Robot position in blue-origin coordinates.
     * @return {@code true} if the robot is on the alliance side of the trench
     *         midpoint and should enter from alliance.
     */
    private static boolean shouldEnterFromAlliance(Translation2d bluePos) {
        return bluePos.getX() < TRENCH_MID_X;
    }

    // ───────────────────── Coordinate normalization ──────────────────────────

    /**
     * Converts a field-relative translation to blue-origin coordinates.
     * On blue alliance this is a no-op; on red it mirrors both axes.
     */
    private static Translation2d toBlueOrigin(Translation2d position) {
        if (Field.isRed()) {
            return new Translation2d(
                    FieldConstants.fieldLength - position.getX(),
                    FieldConstants.fieldWidth - position.getY());
        }
        return position;
    }

    /**
     * Converts field-relative chassis speeds to blue-origin frame.
     * On red alliance both vx and vy are negated to match the mirrored axes.
     */
    private static ChassisSpeeds toBlueOriginSpeeds(ChassisSpeeds speeds) {
        if (Field.isRed()) {
            return new ChassisSpeeds(
                    -speeds.vxMetersPerSecond,
                    -speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond);
        }
        return speeds;
    }
}
