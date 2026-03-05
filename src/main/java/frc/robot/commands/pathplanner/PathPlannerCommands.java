package frc.robot.commands.pathplanner;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.commandGroups.FuelHuntCommand;
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
     */
    public static void registerFuelHuntCommands() {
        addCommand("FuelHunt Choose Return Trench", createScoredTrenchDecisionCommand());
        addCommand("Collect Fuel", Commands.waitSeconds(1.5).withName("CollectFuelWait"));
        addCommand("FuelHunt Full", FuelHuntCommand.create());
    }

    /**
     * Creates a command that uses the {@link FuelScorer} to evaluate fuel
     * clusters and pick the optimal return trench.  Falls back to simple
     * Y-proximity if no confirmed fuels are available.
     */
    private static Command createScoredTrenchDecisionCommand() {
        final double LEFT_TRENCH_Y = 0.615;
        final double RIGHT_TRENCH_Y = 7.428;

        return Commands.defer(() -> {
            try {
                Pose2d currentPose = RobotContainer.m_swerveInstance.getRobotPose();
                Translation2d robotPos = currentPose.getTranslation();

                // Try fuel-map-based scoring
                FuelDetection fuelDet = RobotContainer.m_fuelDetectionInstance;
                String chosenPath;

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
                            String.format("[FuelHunt] Scored return — cluster at Y=%.2f → %s",
                                clusterY, chosenPath), false);
                    } else {
                        // No confirmed fuels — fallback
                        chosenPath = fallbackTrench(robotPos.getY(), LEFT_TRENCH_Y, RIGHT_TRENCH_Y);
                    }
                } else {
                    chosenPath = fallbackTrench(robotPos.getY(), LEFT_TRENCH_Y, RIGHT_TRENCH_Y);
                }

                return AutoBuilder.followPath(PathPlannerPath.fromPathFile(chosenPath));
            } catch (Exception e) {
                DriverStation.reportError("[FuelHunt] Failed to load return trench path: "
                    + e.getMessage(), e.getStackTrace());
                return Commands.none();
            }
        }, Set.of(RobotContainer.m_swerveInstance));
    }

    private static String fallbackTrench(double y, double leftY, double rightY) {
        double distL = Math.abs(y - leftY);
        double distR = Math.abs(y - rightY);
        String chosenPath = (distL <= distR)
            ? "FuelHunt_ReturnLTrench"
            : "FuelHunt_ReturnRTrench";
        DriverStation.reportWarning(
            String.format("[FuelHunt] No confirmed fuels — fallback Y=%.2f → %s", y, chosenPath),
            false);
        return chosenPath;
    }

}
