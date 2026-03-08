package frc.lib.commands;

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

// THIS IS MEANT TO BE EASY TO COPY AND PASTE FROM ONE SEASON TO THE NEXT.
// This just offers a way to store all commands used in PathPlanner in one place, and to easily add new ones as needed on a per-subsystem basis.
// Don't add any subsystem or season-specific commands here, just use this as a template for where to store PathPlanner commands in the future.
public class PathPlannerCommands {
    private static HashMap<String, Command> availableCommands = new HashMap<String, Command>();

    public static void addCommand(String name, Command command) {
        availableCommands.put(name, command);
    }

    public static Map<String, Command> getAvailableCommands() {
        return availableCommands;
    }
}
