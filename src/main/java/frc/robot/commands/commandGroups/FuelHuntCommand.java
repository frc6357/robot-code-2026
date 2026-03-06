package frc.robot.commands.commandGroups;

import static frc.robot.Konstants.AutoConstants.kFuelHuntConstraints;
import static frc.robot.Konstants.FuelHuntConstants.*;

import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.fueldetection.FuelCluster;
import frc.robot.subsystems.fueldetection.FuelDetection;
import frc.robot.subsystems.fueldetection.FuelScorer;

/**
 * Fuel-hunt command: drives through the neutral zone collecting fuel balls.
 *
 * <h2>Architecture</h2>
 * Uses the <b>existing</b> {@link FuelDetection} subsystem (which maintains a
 * persistent {@link frc.robot.subsystems.fueldetection.FuelMap FuelMap} and
 * scores clusters via {@link FuelScorer}) to decide where to drive.
 *
 * <h2>Strategy</h2>
 * <ol>
 *   <li><b>Phase 1 — Pathfind to trench entry</b>: drives to the nearest
 *       trench entry using pathfinding.</li>
 *   <li><b>Phase 2 — Follow outbound trench</b>: follows a pre-planned path
 *       through the trench. The intake (on the back) picks up trench fuel,
 *       and the camera builds up a fuel map of the neutral zone.</li>
 *   <li><b>Phase 3 — Fuel collection loop</b>: repeatedly queries
 *       {@link FuelScorer#bestCluster} for the best fuel cluster, pathfinds
 *       to it, then re-evaluates. Continues until no affordable clusters
 *       remain or the robot has been hunting too long.</li>
 *   <li><b>Phase 4 — Return</b>: pathfinds to the opposite trench and
 *       follows the return path home.</li>
 * </ol>
 *
 * <p>The robot goes OUT through the closest trench and comes BACK through
 * the opposite trench, creating a loop (L→R or R→L).
 *
 * <p>Intake is on the BACK — heading is oriented back-first toward targets.
 */
public class FuelHuntCommand {

    // ==================== Field geometry ====================

    private static final double LEFT_TRENCH_Y  = 0.615;
    private static final double RIGHT_TRENCH_Y = 7.428;

    /** Entry poses — heading 180° = back-first going +X. */
    private static final Pose2d L_ENTRY =
        new Pose2d(4.616, LEFT_TRENCH_Y, Rotation2d.fromDegrees(180));
    private static final Pose2d R_ENTRY =
        new Pose2d(4.627, RIGHT_TRENCH_Y, Rotation2d.fromDegrees(180));

    /** Far end of each trench — used for detour cost calculation. */
    private static final Translation2d L_FAR = new Translation2d(7.5, LEFT_TRENCH_Y);
    private static final Translation2d R_FAR = new Translation2d(7.5, RIGHT_TRENCH_Y);

    // ==================== Pre-planned path names ====================

    private static final String PATH_RET_L  = "FuelHunt_ReturnViaLTrench";
    private static final String PATH_RET_R  = "FuelHunt_ReturnViaRTrench";

    // Tuning constants are in Konstants.FuelHuntConstants (static-imported above).
    // Speed / acceleration are in Konstants.AutoConstants.kFuelHuntConstraints.

    // ==================== Public factory ====================

    /**
     * Creates the fuel collection command.
     * Works from any position on the field.
     * Bind with {@code whileTrue()} so releasing the button cancels.
     */
    public static Command create() {
        return Commands.defer(() -> {
            boolean outLeft = isLeftCloser(getRobotPose().getY());
            return buildSequence(outLeft);
        }, Set.of(RobotContainer.m_swerveInstance))
            .withName("FuelHunt");
    }

    // ==================== Sequence builder ====================

    private static Command buildSequence(boolean outLeft) {
        Pose2d   entry      = outLeft ? L_ENTRY : R_ENTRY;
        String   retPath    = outLeft ? PATH_RET_R  : PATH_RET_L;
        Translation2d retFar = outLeft ? R_FAR       : L_FAR;
        double   retTrenchY = outLeft ? RIGHT_TRENCH_Y : LEFT_TRENCH_Y;
        double   entryTrenchY = outLeft ? LEFT_TRENCH_Y : RIGHT_TRENCH_Y;

        log("Plan: out %s trench → hunt → return %s trench",
            outLeft ? "LEFT" : "RIGHT", outLeft ? "RIGHT" : "LEFT");

        return Commands.sequence(
            // Phase 1: Pathfind to trench entry (back-first)
            logState("PATHFIND_TO_ENTRY"),
            AutoBuilder.pathfindToPose(entry, kFuelHuntConstraints, kEntryGoalEndVel)
                .withName("FH_PathfindToEntry"),

            // Phase 2: Start hunting immediately — no outbound trench path.
            // The camera (facing backward = into the neutral zone) has been
            // building a fuel map during the pathfind phase already.
            logState("HUNTING"),
            buildHuntLoop(retFar, retTrenchY, entryTrenchY, retPath)
                .withName("FH_HuntLoop")
        ).withName("FuelHunt_Full");
    }

    // ==================== Phase 3: hunt loop ====================

    /**
     * Wraps the hunt step with a start-time capture and initial ball count
     * for tracking how many have been collected.
     */
    private static Command buildHuntLoop(Translation2d retFar,
                                          double retTrenchY,
                                          double entryTrenchY,
                                          String retPath) {
        final double[] huntStart = { 0 };
        final int[] legCount = { 0 };
        final int[] startBalls = { 0 };

        return Commands.sequence(
            Commands.runOnce(() -> {
                huntStart[0] = Timer.getFPGATimestamp();
                legCount[0] = 0;
                // Snapshot how many balls remain right now so we can
                // compute collected = startBalls - remaining later.
                FuelDetection det = RobotContainer.m_fuelDetectionInstance;
                if (det != null && det.getSimFuelField() != null) {
                    startBalls[0] = det.getSimFuelField().getRemainingCount();
                } else {
                    startBalls[0] = 200; // fallback for real robot
                }
            }),
            buildHuntStep(retFar, retTrenchY, entryTrenchY, retPath, huntStart, legCount, startBalls)
        );
    }

    /**
     * A single step of the hunt loop. Evaluates fuel state and either:
     *  - Detours to best cluster then recurses, or
     *  - Returns home.
     *
     * <p>Exit conditions (in priority order):
     * <ol>
     *   <li>Safety timeout ({@code kMaxHuntTimeSec})</li>
     *   <li>Collected ≥ {@code kCollectionTarget} balls</li>
     *   <li>No affordable fuel clusters remain</li>
     * </ol>
     */
    private static Command buildHuntStep(Translation2d retFar,
                                          double retTrenchY,
                                          double entryTrenchY,
                                          String retPath,
                                          double[] huntStart,
                                          int[] legCount,
                                          int[] startBalls) {
        return Commands.defer(() -> {
            Translation2d robotPos = getRobotPose().getTranslation();
            double directCost = robotPos.getDistance(retFar);
            double elapsed = Timer.getFPGATimestamp() - huntStart[0];

            // How many balls collected so far?
            int collected = 0;
            FuelDetection det = RobotContainer.m_fuelDetectionInstance;
            if (det != null && det.getSimFuelField() != null) {
                collected = startBalls[0] - det.getSimFuelField().getRemainingCount();
            }
            Logger.recordOutput("FuelHunt/Collected", collected);
            Logger.recordOutput("FuelHunt/Elapsed", elapsed);

            // --- Exit condition 1: safety timeout ---
            if (elapsed > kMaxHuntTimeSec) {
                log("Hunt timeout (%.1fs, %d collected) — returning", elapsed, collected);
                return buildReturnSequence(retPath);
            }

            // --- Exit condition 2: collection target reached ---
            if (collected >= kCollectionTarget) {
                log("Target reached (%d collected) — returning", collected);
                return buildReturnSequence(retPath);
            }

            // Find best affordable fuel cluster
            Optional<FuelCluster> best = findBestAffordableCluster(
                robotPos, retFar, retTrenchY, entryTrenchY, directCost);

            if (best.isPresent()) {
                FuelCluster cluster = best.get();
                Translation2d fuelPos = clamp(cluster.getCentroid());
                double detourExtra = robotPos.getDistance(fuelPos)
                                   + fuelPos.getDistance(retFar) - directCost;

                legCount[0]++;
                log("Leg %d → (%.1f,%.1f) n=%d +%.1fm",
                    legCount[0], fuelPos.getX(), fuelPos.getY(),
                    cluster.getCount(), detourExtra);

                // Log target for AdvantageScope
                Logger.recordOutput("FuelHunt/Target",
                    new Pose2d[] { new Pose2d(fuelPos, new Rotation2d()) });
                Logger.recordOutput("FuelHunt/TargetCount", cluster.getCount());
                Logger.recordOutput("FuelHunt/Leg", legCount[0]);

                // Back-first heading so intake leads
                Rotation2d heading = backFirst(robotPos, fuelPos);
                Pose2d fuelTarget = new Pose2d(fuelPos, heading);

                return Commands.sequence(
                    // Race the pathfind against a proximity check so the robot
                    // moves on to the next leg the instant it's close enough —
                    // no pause while PathPlanner plans the next path.
                    AutoBuilder.pathfindToPose(fuelTarget, kFuelHuntConstraints, kFuelGoalEndVel)
                        .raceWith(
                            // End this leg as soon as robot is within kFuelProximityM
                            Commands.waitUntil(() ->
                                getRobotPose().getTranslation().getDistance(fuelPos) < kFuelProximityM)
                        )
                        .withTimeout(kLegTimeoutSec)
                        .withName("FH_GoToFuel_" + legCount[0]),
                    // Immediately start next leg — no stop, no pause
                    buildHuntStep(retFar, retTrenchY, entryTrenchY, retPath, huntStart, legCount, startBalls)
                );
            }

            // No affordable fuel — return
            log("No affordable fuel — returning");
            Logger.recordOutput("FuelHunt/Target", new Pose2d[0]);
            return buildReturnSequence(retPath);

        }, Set.of(RobotContainer.m_swerveInstance));
    }

    // ==================== Phase 4: Return ====================

    private static Command buildReturnSequence(String retPath) {
        return Commands.sequence(
            logState("RETURNING"),
            buildReturn(retPath).withName("FH_Return")
        );
    }

    // ==================== Fuel evaluation ====================

    /**
     * Uses {@link FuelScorer} to find the best cluster whose detour cost
     * is within budget. All scoring (density, proximity, path cost,
     * confidence) is handled by the existing scorer.
     */
    private static Optional<FuelCluster> findBestAffordableCluster(
            Translation2d robotPos, Translation2d retFar,
            double retTrenchY, double entryTrenchY, double directCost) {

        FuelDetection det = RobotContainer.m_fuelDetectionInstance;
        if (det == null) return Optional.empty();

        List<FuelScorer.ScoredCluster> ranked =
            FuelScorer.rankClusters(det.getFuelMap().getConfirmedFuels(),
                                    robotPos, retTrenchY);

        Logger.recordOutput("FuelHunt/ClustersFound", ranked.size());
        Logger.recordOutput("FuelHunt/ConfirmedFuels",
            det.getFuelMap().getConfirmedFuels().size());

        for (FuelScorer.ScoredCluster sc : ranked) {
            Translation2d c = sc.getCluster().getCentroid();
            // Skip clusters outside neutral zone X boundaries
            if (c.getX() < kNzMinX || c.getX() > kNzMaxX) continue;
            // Skip clusters too far from the entry trench (lateral distance)
            if (Math.abs(c.getY() - entryTrenchY) > kMaxTrenchLateralM) continue;
            double detour = robotPos.getDistance(c) + c.getDistance(retFar);
            if (detour - directCost <= kMaxDetourExtraM) {
                Logger.recordOutput("FuelHunt/ChosenScore", sc.getScore());
                return Optional.of(sc.getCluster());
            }
        }
        return Optional.empty();
    }

    // ==================== Path helpers ====================

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
            MathUtil.clamp(pos.getX(), kFieldMinX, kFieldMaxX),
            MathUtil.clamp(pos.getY(), kFieldMinY, kFieldMaxY));
    }

    private static void log(String fmt, Object... args) {
        String msg = "[FuelHunt] " + String.format(fmt, args);
        DriverStation.reportWarning(msg, false);
        Logger.recordOutput("FuelHunt/Status", msg);
    }

    private static Command logState(String state) {
        return Commands.runOnce(() -> {
            Logger.recordOutput("FuelHunt/State", state);
            log("State → %s", state);
        }).withName("FH_Log_" + state);
    }
}
