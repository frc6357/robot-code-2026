package frc.robot.commands.commandGroups;

import static frc.robot.Konstants.AutoConstants.kDefaultPathfindingConstraints;

import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.auto.Pathfinder;
import frc.lib.auto.Pathfollower;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.fueldetection.FuelCluster;
import frc.robot.subsystems.fueldetection.FuelDetection;
import frc.robot.subsystems.fueldetection.FuelScorer;

/**
 * A command group that drives the robot through the optimal trench to the
 * neutral zone, picks up the best-scored fuel cluster, then returns through
 * the nearest trench back to the scoring position.
 *
 * <h2>Usage</h2>
 * <ul>
 *   <li><b>Teleop:</b> Bind to a driver button with {@code whileTrue()}</li>
 *   <li><b>Auto:</b> Schedule directly or register as a PathPlanner NamedCommand</li>
 * </ul>
 *
 * <h2>Path sequence</h2>
 * The command dynamically selects from pre-built PathPlanner paths:
 * <ol>
 *   <li>{@code FuelHunt_ToLTrench} — drive from scoring wall toward the left trench</li>
 *   <li>{@code FuelHunt_ThroughLTrench} — through trench to neutral zone</li>
 *   <li>Pause briefly to collect fuel (intaking state)</li>
 *   <li>{@code FuelHunt_ReturnLTrench} or {@code FuelHunt_ReturnRTrench} — chosen
 *       based on which trench is closest to the best fuel cluster</li>
 * </ol>
 */
public class FuelHuntCommand {

    // ---- Field geometry (meters) ----
    private static final double LEFT_TRENCH_Y  = 0.615;
    private static final double RIGHT_TRENCH_Y = 7.428;

    // ---- Path names ----
    private static final String PATH_TO_L_TRENCH      = "FuelHunt_ToLTrench";
    private static final String PATH_THROUGH_L_TRENCH  = "FuelHunt_ThroughLTrench";
    private static final String PATH_RETURN_L_TRENCH   = "FuelHunt_ReturnLTrench";
    private static final String PATH_RETURN_R_TRENCH   = "FuelHunt_ReturnRTrench";

    /** Time (seconds) to wait at the neutral zone for fuel intake. */
    private static final double COLLECT_WAIT_SECONDS = 1.5;

    // ==================== Public factory methods ====================

    /**
     * Creates the full FuelHunt command group.  Can be used as-is for auto,
     * or bound to a driver button with {@code whileTrue()}.
     *
     * <p>The command requires the swerve subsystem (via the path-following
     * commands) so it will be properly interrupted when the driver releases
     * the button.
     *
     * @return The full FuelHunt sequential command group
     */
    public static Command create() {
        return Commands.sequence(
            // 1. Drive to the left trench entry (pathfind to start, then follow)
            Pathfinder.PathfindThenFollowPathCommand(PATH_TO_L_TRENCH, kDefaultPathfindingConstraints)
                .withName("FuelHunt_ToLTrench"),

            // 2. Drive through the left trench to the neutral zone
            Pathfollower.FollowPathCommand(PATH_THROUGH_L_TRENCH)
                .withName("FuelHunt_ThroughLTrench"),

            // 3. Brief pause to let intake grab fuel
            Commands.waitSeconds(COLLECT_WAIT_SECONDS)
                .withName("FuelHunt_CollectPause"),

            // 4. Decide which trench to return through (scored dynamically)
            createScoredReturnCommand()
                .withName("FuelHunt_ScoredReturn")
        ).withName("FuelHunt");
    }

    /**
     * Creates a version that loops — drives out, collects, returns, and
     * repeats until cancelled.  Ideal for holding a button in teleop.
     *
     * @return Repeating FuelHunt command
     */
    public static Command createRepeating() {
        return create().repeatedly().withName("FuelHunt_Repeating");
    }

    // ==================== Internal logic ====================

    /**
     * Creates the deferred return command that evaluates the fuel map at
     * runtime and picks the return trench closest to the best fuel cluster
     * (or falls back to proximity-based trench selection).
     */
    private static Command createScoredReturnCommand() {
        return Commands.defer(() -> {
            try {
                String chosenPath = chooseBestReturnPath();
                DriverStation.reportWarning("[FuelHunt] Returning via: " + chosenPath, false);
                return AutoBuilder.followPath(PathPlannerPath.fromPathFile(chosenPath));
            } catch (Exception e) {
                DriverStation.reportError("[FuelHunt] Failed to load return path: "
                    + e.getMessage(), e.getStackTrace());
                return Commands.none();
            }
        }, Set.of(RobotContainer.m_swerveInstance));
    }

    /**
     * Evaluates the fuel map and robot position to decide which trench to
     * return through.
     *
     * <ol>
     *   <li>If the fuel map has confirmed fuels, score and cluster them.
     *       The best cluster's Y position determines the return trench.</li>
     *   <li>If no confirmed fuels are visible, fall back to the trench
     *       closest to the robot's current Y coordinate.</li>
     * </ol>
     */
    private static String chooseBestReturnPath() {
        SKSwerve swerve = RobotContainer.m_swerveInstance;
        FuelDetection fuelDetection = RobotContainer.m_fuelDetectionInstance;

        Pose2d robotPose = swerve.getRobotPose();
        Translation2d robotPos = robotPose.getTranslation();

        // Try fuel-based scoring
        if (fuelDetection != null) {
            Optional<FuelCluster> best = FuelScorer.bestCluster(
                fuelDetection.getFuelMap().getConfirmedFuels(),
                robotPos
            );

            if (best.isPresent()) {
                FuelCluster cluster = best.get();
                double clusterY = cluster.getCentroid().getY();

                DriverStation.reportWarning(
                    String.format("[FuelHunt] Best cluster: %s (score via FuelScorer)", cluster),
                    false);

                return pickTrenchByY(clusterY);
            }
        }

        // Fallback: use robot's current Y
        DriverStation.reportWarning(
            "[FuelHunt] No confirmed fuels — using robot Y for trench selection", false);
        return pickTrenchByY(robotPos.getY());
    }

    /**
     * Picks the return trench path based on a Y coordinate.
     * Whichever trench center line is closer to the given Y wins.
     */
    private static String pickTrenchByY(double y) {
        double distToLeft  = Math.abs(y - LEFT_TRENCH_Y);
        double distToRight = Math.abs(y - RIGHT_TRENCH_Y);

        if (distToLeft <= distToRight) {
            return PATH_RETURN_L_TRENCH;
        } else {
            return PATH_RETURN_R_TRENCH;
        }
    }
}
