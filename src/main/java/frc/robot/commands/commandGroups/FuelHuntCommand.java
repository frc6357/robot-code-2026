package frc.robot.commands.commandGroups;

import static frc.robot.Konstants.AutoConstants.kFuelHuntConstraints;

import java.util.List;
import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.fueldetection.FuelCluster;
import frc.robot.subsystems.fueldetection.FuelDetection;
import frc.robot.subsystems.fueldetection.FuelScorer;

/**
 * Cross-trench fuel hunting command.
 *
 * <p>The robot goes OUT through the closest trench and comes BACK through
 * the opposite trench, creating a deterministic loop:
 * <ul>
 *   <li>L trench out → R trench back</li>
 *   <li>R trench out → L trench back</li>
 * </ul>
 *
 * <p>Because the return trench is always known, path cost for every fuel
 * cluster can be calculated accurately against the return trench far-end.
 * Fuel collection is an optional, budget-limited detour in the neutral
 * zone between the outbound exit and the return trench far-end.
 *
 * <p>There are ZERO stops in the entire sequence — the robot flows
 * continuously from start to finish.  If it sees fuel on the way it
 * weighs the detour cost and decides whether to go or skip.
 *
 * <p>Intake is on the BACK — when detouring to fuel the heading is
 * flipped 180° so the back leads into the ball.
 */
public class FuelHuntCommand {

    // ---- Field geometry (meters) ----
    private static final double LEFT_TRENCH_Y  = 0.615;
    private static final double RIGHT_TRENCH_Y = 7.428;

    // Trench entries (alliance side) — 180° so intake (back) leads into trench
    private static final Pose2d L_ENTRY =
        new Pose2d(4.616, LEFT_TRENCH_Y, Rotation2d.fromDegrees(180));
    private static final Pose2d R_ENTRY =
        new Pose2d(4.627, RIGHT_TRENCH_Y, Rotation2d.fromDegrees(180));

    // Trench far ends (neutral zone side) — where outbound exits / return starts
    private static final Translation2d L_FAR = new Translation2d(7.5, LEFT_TRENCH_Y);
    private static final Translation2d R_FAR = new Translation2d(7.5, RIGHT_TRENCH_Y);

    // Path file names
    private static final String PATH_OUT_L    = "FuelHunt_OutLTrench";
    private static final String PATH_OUT_R    = "FuelHunt_OutRTrench";
    private static final String PATH_RET_L    = "FuelHunt_ReturnViaLTrench";
    private static final String PATH_RET_R    = "FuelHunt_ReturnViaRTrench";

    // ---- Field bounds for clamping ----
    private static final double FIELD_MIN_X = 0.5;
    private static final double FIELD_MAX_X = 16.5;
    private static final double FIELD_MIN_Y = 0.3;
    private static final double FIELD_MAX_Y = 7.8;

    // ---- Tuning ----
    /** Max extra meters a single fuel detour may add vs going straight to return. */
    private static final double MAX_DETOUR_EXTRA_M = 8.0;

    /** Goal-end velocity when arriving at fuel — keep rolling through. */
    private static final double FUEL_GOAL_END_VEL = 1.2;

    /** Goal-end velocity when arriving at trench entry — keep rolling. */
    private static final double ENTRY_GOAL_END_VEL = 1.5;

    /** How close (meters) the robot must be to retFar to stop looking for fuel. */
    private static final double CLOSE_ENOUGH_M = 2.0;

    // ==================== Public factory ====================

    /**
     * Creates the full FuelHunt command.  Works from any position on the
     * field.  Bind with {@code whileTrue()} so releasing the button cancels.
     */
    public static Command create() {
        return Commands.defer(() -> {
            boolean outLeft = isLeftCloser(getRobotPose().getY());
            return buildSequence(outLeft);
        }, Set.of(RobotContainer.m_swerveInstance))
            .withName("FuelHunt");
    }

    // ==================== Sequence builder ====================

    /**
     * Builds the full 3-phase sequence for the given outbound direction.
     *
     * @param outLeft true = go out L trench, return R trench.
     *                false = go out R trench, return L trench.
     */
    private static Command buildSequence(boolean outLeft) {
        Pose2d   entry      = outLeft ? L_ENTRY : R_ENTRY;
        String   outPath    = outLeft ? PATH_OUT_L  : PATH_OUT_R;
        String   retPath    = outLeft ? PATH_RET_R  : PATH_RET_L;   // OPPOSITE
        Translation2d retFar = outLeft ? R_FAR       : L_FAR;        // OPPOSITE
        double   retTrenchY = outLeft ? RIGHT_TRENCH_Y : LEFT_TRENCH_Y; // OPPOSITE

        log("Out %s trench, return %s trench",
            outLeft ? "L" : "R", outLeft ? "R" : "L");

        return Commands.sequence(
            // Phase 1 — pathfind to nearest trench entry
            AutoBuilder.pathfindToPose(entry, kFuelHuntConstraints, ENTRY_GOAL_END_VEL)
                .withName("FuelHunt_ToEntry"),

            // Phase 2 — follow outbound trench path (intake picks up trench fuel)
            buildFollowPath(outPath)
                .withName("FuelHunt_Outbound"),

            // Phase 3 — evaluate fuel, detour or go straight to return trench
            deferFuelOrReturn(retFar, retTrenchY, retPath)
                .withName("FuelHunt_NeutralDecision")
        );
    }

    // ==================== Phase 3: the neutral-zone decision ====================

    /**
     * Deferred command that repeatedly evaluates fuel clusters.  After each
     * pickup it re-evaluates — if more affordable fuel exists it detours
     * again, otherwise it heads home.  This creates a continuous sweeping
     * motion across the neutral zone, collecting fuel on the way to the
     * opposite trench.
     */
    private static Command deferFuelOrReturn(Translation2d retFar,
                                             double retTrenchY,
                                             String retPath) {
        return Commands.defer(() -> {
            Translation2d robotPos = getRobotPose().getTranslation();
            double directCost = robotPos.getDistance(retFar);

            // If we're already close to the return trench far-end, just go home
            if (directCost < CLOSE_ENOUGH_M) {
                log("Close to return trench, heading home");
                return buildReturn(retPath).withName("FuelHunt_Return");
            }

            Optional<FuelCluster> best = findAffordable(robotPos, retFar, retTrenchY, directCost);

            if (best.isPresent()) {
                FuelCluster cluster = best.get();
                Translation2d fuelPos = clamp(cluster.getCentroid());
                double extra = robotPos.getDistance(fuelPos)
                             + fuelPos.getDistance(retFar) - directCost;

                log("Detour to %s (n=%d, +%.1fm)", fmt(fuelPos), cluster.getCount(), extra);

                // Head TOWARD the fuel — swerve holonomic controller handles rotation
                Rotation2d heading = backFirst(robotPos, fuelPos);
                Pose2d fuelTarget = new Pose2d(fuelPos, heading);

                return Commands.sequence(
                    AutoBuilder.pathfindToPose(fuelTarget, kFuelHuntConstraints, FUEL_GOAL_END_VEL)
                        .withName("FuelHunt_GoToFuel"),
                    // Re-evaluate: more fuel or head home?
                    deferFuelOrReturn(retFar, retTrenchY, retPath)
                        .withName("FuelHunt_ReEvaluate")
                );
            }

            // No affordable fuel — go straight to return trench
            log("No affordable fuel, returning");
            return buildReturn(retPath).withName("FuelHunt_Return");

        }, Set.of(RobotContainer.m_swerveInstance));
    }

    // ==================== Fuel evaluation ====================

    /**
     * Returns the best cluster whose detour cost exceeds the direct cost
     * by at most {@link #MAX_DETOUR_EXTRA_M}.  Scores clusters against
     * the known return trench Y.
     */
    private static Optional<FuelCluster> findAffordable(
            Translation2d robotPos, Translation2d retFar,
            double retTrenchY, double directCost) {

        FuelDetection det = RobotContainer.m_fuelDetectionInstance;
        if (det == null) return Optional.empty();

        List<FuelScorer.ScoredCluster> ranked =
            FuelScorer.rankClusters(det.getFuelMap().getConfirmedFuels(),
                                    robotPos, retTrenchY);

        for (FuelScorer.ScoredCluster sc : ranked) {
            Translation2d c = sc.getCluster().getCentroid();
            double detour = robotPos.getDistance(c) + c.getDistance(retFar);
            if (detour - directCost <= MAX_DETOUR_EXTRA_M) {
                return Optional.of(sc.getCluster());
            }
        }
        return Optional.empty();
    }

    // ==================== Path helpers ====================

    /** Follow a pre-built PathPlanner path file. */
    private static Command buildFollowPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError(
                "[FuelHunt] Failed to load " + pathName + ": " + e.getMessage(),
                e.getStackTrace());
            return Commands.none();
        }
    }

    /** Pathfind to the start of the return path, then follow it home. */
    private static Command buildReturn(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return AutoBuilder.pathfindThenFollowPath(path, kFuelHuntConstraints);
        } catch (Exception e) {
            DriverStation.reportError(
                "[FuelHunt] Failed to load " + pathName + ": " + e.getMessage(),
                e.getStackTrace());
            return Commands.none();
        }
    }

    // ==================== Helpers ====================

    /** Back-first heading: 180° from direction of travel (intake on back). */
    private static Rotation2d backFirst(Translation2d from, Translation2d to) {
        double angle = Math.atan2(to.getY() - from.getY(), to.getX() - from.getX());
        return Rotation2d.fromRadians(angle + Math.PI);
    }

    private static boolean isLeftCloser(double y) {
        return Math.abs(y - LEFT_TRENCH_Y) <= Math.abs(y - RIGHT_TRENCH_Y);
    }

    private static Pose2d getRobotPose() {
        return RobotContainer.m_swerveInstance.getRobotPose();
    }

    private static Translation2d clamp(Translation2d pos) {
        return new Translation2d(
            MathUtil.clamp(pos.getX(), FIELD_MIN_X, FIELD_MAX_X),
            MathUtil.clamp(pos.getY(), FIELD_MIN_Y, FIELD_MAX_Y));
    }

    private static void log(String fmt, Object... args) {
        DriverStation.reportWarning("[FuelHunt] " + String.format(fmt, args), false);
    }

    private static String fmt(Pose2d p) {
        return String.format("(%.2f, %.2f, %.0fdeg)",
            p.getX(), p.getY(), p.getRotation().getDegrees());
    }

    private static String fmt(Translation2d t) {
        return String.format("(%.2f, %.2f)", t.getX(), t.getY());
    }
}
