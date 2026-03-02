package frc.robot.commands.pathplanner;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

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
     * Registers the FuelHunt trench decision command as a PathPlanner NamedCommand.
     * This must be called after the swerve subsystem is created.
     */
    public static void registerFuelHuntCommands() {
        addCommand("FuelHunt Choose Return Trench", createTrenchDecisionCommand());
    }

    /**
     * Creates a command that reads the robot's current pose and
     * decides whether to return through the left trench or the right trench based on which
     * trench entrance is closer to the robot's current Y coordinate.
     */
    
    private static Command createTrenchDecisionCommand() {
        // Y coordinates of the left/right trench centers (meters)
        final double LEFT_TRENCH_Y = 0.615;
        final double RIGHT_TRENCH_Y = 7.428;

        return Commands.defer(() -> {
            try {
                Pose2d currentPose = RobotContainer.m_swerveInstance.getRobotPose();
                double currentY = currentPose.getY();

                double distToLeft = Math.abs(currentY - LEFT_TRENCH_Y);
                double distToRight = Math.abs(currentY - RIGHT_TRENCH_Y);

                String chosenPath;
                if (distToLeft <= distToRight) {
                    chosenPath = "FuelHunt_ReturnLTrench";
                    DriverStation.reportWarning("[FuelHunt] Returning through LEFT trench (Y=" 
                        + String.format("%.2f", currentY) + ")", false);
                } else {
                    chosenPath = "FuelHunt_ReturnRTrench";
                    DriverStation.reportWarning("[FuelHunt] Returning through RIGHT trench (Y=" 
                        + String.format("%.2f", currentY) + ")", false);
                }

                return AutoBuilder.followPath(PathPlannerPath.fromPathFile(chosenPath));
            } catch (Exception e) {
                DriverStation.reportError("[FuelHunt] Failed to load return trench path: " 
                    + e.getMessage(), e.getStackTrace());
                return Commands.none();
            }
        }, Set.of(RobotContainer.m_swerveInstance));
    }

}
