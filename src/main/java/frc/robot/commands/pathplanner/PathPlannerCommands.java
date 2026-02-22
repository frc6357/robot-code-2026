package frc.robot.commands.pathplanner;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;

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

}
