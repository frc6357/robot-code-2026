package frc.robot.commands.pathplanner;

import static frc.robot.Konstants.AutoConstants.kFuelHuntConstraints;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.commandGroups.FuelHuntCommand;
// FuelAutoCoordinator removed — corridor system replaced by FuelHuntCommand
import frc.robot.subsystems.fueldetection.FuelCluster;
import frc.robot.subsystems.fueldetection.FuelDetection;
import frc.robot.subsystems.fueldetection.FuelScorer;

/**
 * A helper class to store and manage all commands that are used in PathPlanner autos and paths.<p>
 * This helps avoid a pileup of boilerplate code in {@link frc.robot.RobotContainer} for PathPlanner autos, 
 * and also allows for easier management of commands used in PathPlanner per subsystem. <p>
 * When a subsystem implements the {@link frc.lib.subsystems.PathplannerSubsystem} interface, it can add commands 
 * to this class that are used in PathPlanner autos, and then those commands will be registered to the NamedCommands.
 */
public class PathPlannerCommands {
    private static HashMap<String, Command> availableCommands = new HashMap<String, Command>();

    public static void addCommand(String name, Command command) {
        availableCommands.put(name, command);
    }

    public static Map<String, Command> getAvailableCommands() {
        return availableCommands;
    }

    /**
     * Registers all FuelHunt-related commands as PathPlanner NamedCommands.
     * This must be called after the swerve and fuel detection subsystems are created.
     *
     * <p>The "FuelHunt Full" command is now fully dynamic — it pathfinds to the
     * closest trench, goes through, pathfinds to the best fuel cluster, and
     * returns through the closest trench.  Works from any starting position.
     */
    public static void registerFuelHuntCommands() {
        // Primary fuel collection command — pathfinds to clusters using FuelScorer
        addCommand("FuelHunt Full", FuelHuntCommand.create());

        // Simple collect pause — can be inserted in any auto sequence
        addCommand("Collect Fuel", Commands.waitSeconds(1.5).withName("CollectFuelWait"));

        // Scored return trench decision — used by the pre-planned FuelHunt.auto
        addCommand("FuelHunt Choose Return Trench", createScoredReturnCommand());
    }

    /**
     * Creates a deferred command that picks the optimal return trench based on
     * fuel scoring, then pathfinds to its start and follows the return path.
     * Used by the pre-planned FuelHunt.auto sequence.
     */
    private static Command createScoredReturnCommand() {
        final double LEFT_TRENCH_Y = 0.615;
        final double RIGHT_TRENCH_Y = 7.428;

        return Commands.defer(() -> {
            try {
                Translation2d robotPos = RobotContainer.m_swerveInstance.getRobotPose().getTranslation();
                String chosenPath;

                FuelDetection fuelDet = RobotContainer.m_fuelDetectionInstance;
                if (fuelDet != null) {
                    Optional<FuelCluster> best = FuelScorer.bestCluster(
                        fuelDet.getFuelMap().getConfirmedFuels(), robotPos);

                    if (best.isPresent()) {
                        double clusterY = best.get().getCentroid().getY();
                        double distL = Math.abs(clusterY - LEFT_TRENCH_Y);
                        double distR = Math.abs(clusterY - RIGHT_TRENCH_Y);
                        chosenPath = (distL <= distR)
                            ? "FuelHunt_ReturnLTrench"
                            : "FuelHunt_ReturnRTrench";
                        DriverStation.reportWarning(
                            String.format("[FuelHunt Auto] Scored return — cluster Y=%.2f → %s",
                                clusterY, chosenPath), false);
                    } else {
                        chosenPath = closestTrench(robotPos.getY(), LEFT_TRENCH_Y, RIGHT_TRENCH_Y);
                    }
                } else {
                    chosenPath = closestTrench(robotPos.getY(), LEFT_TRENCH_Y, RIGHT_TRENCH_Y);
                }

                PathPlannerPath path = PathPlannerPath.fromPathFile(chosenPath);
                return AutoBuilder.pathfindThenFollowPath(path, kFuelHuntConstraints);
            } catch (Exception e) {
                DriverStation.reportError("[FuelHunt Auto] Return path error: "
                    + e.getMessage(), e.getStackTrace());
                return Commands.none();
            }
        }, Set.of(RobotContainer.m_swerveInstance));
    }

    private static String closestTrench(double y, double leftY, double rightY) {
        String chosen = (Math.abs(y - leftY) <= Math.abs(y - rightY))
            ? "FuelHunt_ReturnLTrench"
            : "FuelHunt_ReturnRTrench";
        DriverStation.reportWarning(
            String.format("[FuelHunt Auto] No fuels — fallback Y=%.2f → %s", y, chosen), false);
        return chosen;
    }
}
