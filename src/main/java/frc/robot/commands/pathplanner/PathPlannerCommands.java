package frc.robot.commands.pathplanner;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.StateHandler.MacroState;

public class PathPlannerCommands {
    public HashMap<String, Command> availableCommands = new HashMap<String, Command>();

    public PathPlannerCommands(RobotContainer container) {
        if(container.m_stateHandlerContainer.isPresent()) {
                availableCommands.put("Request Idle State", container.m_stateHandlerContainer.get().requestStateCommand(MacroState.IDLE));
                availableCommands.put("Request Scoring State", container.m_stateHandlerContainer.get().requestStateCommand(MacroState.SCORING));
                availableCommands.put("Request Shuttling State", container.m_stateHandlerContainer.get().requestStateCommand(MacroState.SHUTTLING));
                availableCommands.put("Request Intaking State", container.m_stateHandlerContainer.get().requestStateCommand(MacroState.INTAKING));
                availableCommands.put("Request Climbing State", container.m_stateHandlerContainer.get().requestStateCommand(MacroState.CLIMBING));
                availableCommands.put("Request Steady Stream Scoring State", container.m_stateHandlerContainer.get().requestStateCommand(MacroState.STEADY_STREAM_SCORING));
                availableCommands.put("Request Steady Stream Shuttling State", container.m_stateHandlerContainer.get().requestStateCommand(MacroState.STEADY_STREAM_SHUTTLING));

                availableCommands.put("Force Idle State", container.m_stateHandlerContainer.get().setCurrentStateCommand(MacroState.IDLE));
                availableCommands.put("Force Scoring State", container.m_stateHandlerContainer.get().setCurrentStateCommand(MacroState.SCORING));
                availableCommands.put("Force Shuttling State", container.m_stateHandlerContainer.get().setCurrentStateCommand(MacroState.SHUTTLING));
                availableCommands.put("Force Intaking State", container.m_stateHandlerContainer.get().setCurrentStateCommand(MacroState.INTAKING));
                availableCommands.put("Force Climbing State", container.m_stateHandlerContainer.get().setCurrentStateCommand(MacroState.CLIMBING));
                availableCommands.put("Force Steady Stream Scoring State", container.m_stateHandlerContainer.get().setCurrentStateCommand(MacroState.STEADY_STREAM_SCORING));
                availableCommands.put("Force Steady Stream Shuttling State", container.m_stateHandlerContainer.get().setCurrentStateCommand(MacroState.STEADY_STREAM_SHUTTLING));
        }
    }
}
