package frc.robot;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.subsystems.PathplannerSubsystem;
import frc.robot.StateHandler.MacroState.Status;
import frc.robot.commands.pathplanner.PathPlannerCommands;

/**
 * A class to handle large-scale robot states (macros) such as launching, intaking, climbing, and idling.
 * Each macro state has an associated status to indicate its current condition.
 * A MacroState can still have its status updated while it is not the current nor desired state.
 */
public class StateHandler extends SubsystemBase implements PathplannerSubsystem{
    
    public enum MacroState {
        IDLE(Status.READY),
        SCORING(Status.OFF),
        SHUTTLING(Status.OFF),
        INTAKING(Status.OFF),
        CLIMBING(Status.OFF),
        STEADY_STREAM_SCORING(Status.OFF),
        STEADY_STREAM_SHUTTLING(Status.OFF);
        
        private MacroState(Status status) {
            this.status = status;
        }
        
        private Status status;

        public Status getStatus() {
            return status;
        }
        public void setStatus(Status status) {
            this.status = status;
        }
        
        // Similar to WPILib Command structure
        public enum Status {
            OFF,
            WAITING,
            READY,
            STOPPING
        }
    }
    private SendableChooser<MacroState> stateChooser = new SendableChooser<MacroState>();        

    private static MacroState currentState = MacroState.IDLE;
    private static MacroState requestedState = MacroState.IDLE;

    private MacroState previousChosenState = MacroState.IDLE;

    public StateHandler() {
        // Reset all states to default on construction
        for (MacroState state : MacroState.values()) {
            state.setStatus(state == MacroState.IDLE ? MacroState.Status.READY : MacroState.Status.OFF);
        }

        stateChooser.setDefaultOption("IDLE", MacroState.IDLE);
        stateChooser.addOption("SCORING", MacroState.SCORING);
        stateChooser.addOption("INTAKING", MacroState.INTAKING);
        stateChooser.addOption("SHUTTLING", MacroState.SHUTTLING);
        stateChooser.addOption("CLIMBING", MacroState.CLIMBING);
        stateChooser.addOption("SS_SCORING", MacroState.STEADY_STREAM_SCORING);
        stateChooser.addOption("SS_SHUTTLING", MacroState.STEADY_STREAM_SHUTTLING);

        SmartDashboard.putData("StateChooser", stateChooser);
    }

    /**
     * Handles changes in the desired state by stopping the current state and initializing the desired state.
     * This method should be called periodically to ensure state transitions are managed.
     */
    private void handleStateTransition() {
        if(requestedState != currentState) {
            currentState.setStatus(Status.STOPPING);
            currentState = requestedState;
            currentState.setStatus(Status.WAITING);
        }
    }

    @Override
    public void periodic() {
        handleStateTransition();
        if(stateChooser.getSelected() != previousChosenState) {
            setCurrentState(stateChooser.getSelected());
            previousChosenState = stateChooser.getSelected();
        }

        SmartDashboard.putData("StateHandler", this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Current State", () -> getCurrentState().name(), null);
        builder.addStringProperty("Requested State", () -> getRequestedState().name(), null);

        for(MacroState state : MacroState.values()) {
            builder.addStringProperty(state.name() + " Status", () -> state.getStatus().name(), null);
        }

    }

    /**
     * Gets the current state.
     * @return The current MacroState.
     */
    public MacroState getCurrentState() {
        return currentState;
    }
    
    /**
     * Forcefully sets the current state and clears the desired state.
     * @param state The new current MacroState.
     */
    public void setCurrentState(MacroState state) {
        currentState = state;
        clearRequestedState();
    }

    public Command setCurrentStateCommand(MacroState state) {
        return runOnce(() -> setCurrentState(state)).withName("Force" + state.name());
    }

    /**
     * Gets the desired state.
     * @return The desired MacroState.
     */
    public MacroState getRequestedState() {
        return requestedState;
    }

    /**
     * Sets the desired state.
     * @param state The desired MacroState.
     */
    public void requestState(MacroState state) {
        requestedState = state;
    }

    public Command requestStateCommand(MacroState state) {
        return runOnce(() -> requestState(state)).withName("Request" + state.name());
    }

    /**
     * Clears the desired MacroState, setting it to whatever the current state is.
     */
    public void clearRequestedState() {
        requestedState = currentState;
    }

    public Command clearRequestedStateCommand() {
        return runOnce(this::clearRequestedState).withName("ClearRequestedState");
    }

    public MacroState.Status getStatusOf(MacroState state) {
        return state.getStatus();
    }
    
    public void setStatusOf(MacroState state, MacroState.Status status) {
        state.setStatus(status);
    }

    public void removeIntakeFromRequestedState() {
        if(requestedState == MacroState.INTAKING) {
            requestedState = MacroState.IDLE;
        }
        if(requestedState == MacroState.STEADY_STREAM_SCORING) {
            requestedState = MacroState.SCORING;
        }
        if(requestedState == MacroState.STEADY_STREAM_SHUTTLING) {
            requestedState = MacroState.SHUTTLING;
        }
    }

    public Command removeIntakeFromRequestedStateCommand() {
        return runOnce(this::removeIntakeFromRequestedState).withName("RemoveIntakeFromRequestedState");
    }

    public void addIntakeToRequestedState() {
        if(requestedState == MacroState.IDLE) {
            requestedState = MacroState.INTAKING;
        }
        if(requestedState == MacroState.SCORING) {
            requestedState = MacroState.STEADY_STREAM_SCORING;
        }
        if(requestedState == MacroState.SHUTTLING) {
            requestedState = MacroState.STEADY_STREAM_SHUTTLING;
        }
    }

    public Command addIntakeToRequestedStateCommand() {
        return runOnce(this::addIntakeToRequestedState).withName("AddIntakeToRequestedState");
    }

    public void toggleIntakeInRequestedState() {
        if(requestedState == MacroState.INTAKING || requestedState == MacroState.STEADY_STREAM_SCORING || requestedState == MacroState.STEADY_STREAM_SHUTTLING) {
            removeIntakeFromRequestedState();
        } else {
            addIntakeToRequestedState();
        }
    }

    public Command toggleIntakeInRequestedStateCommand() {
        return runOnce(this::toggleIntakeInRequestedState).withName("ToggleIntakeInRequestedState");
    }

    public void removeScoringFromRequestedState() {
        if(requestedState == MacroState.SCORING) {
            requestedState = MacroState.IDLE;
        }
        else if(requestedState == MacroState.STEADY_STREAM_SCORING) {
            requestedState = MacroState.INTAKING;
        }
    }

    public void addScoringToRequestedState() {
        if(requestedState == MacroState.IDLE) {
            requestedState = MacroState.SCORING;
        }
        else if(requestedState == MacroState.INTAKING) {
            requestedState = MacroState.STEADY_STREAM_SCORING;
        }
        else if(requestedState == MacroState.SHUTTLING) {
            requestedState = MacroState.SCORING;
        }
        else if(requestedState == MacroState.STEADY_STREAM_SHUTTLING) {
            requestedState = MacroState.STEADY_STREAM_SCORING;
        }
    }

    public void requestScoring() {
        addScoringToRequestedState();
    }

    public Command requestScoringCommand() {
        return runOnce(this::requestScoring).withName("RequestScoring");
    }

    public void removeShuttlingFromRequestedState() {
        if(requestedState == MacroState.SHUTTLING) {
            requestedState = MacroState.IDLE;
        }
        else if(requestedState == MacroState.STEADY_STREAM_SHUTTLING) {
            requestedState = MacroState.INTAKING;
        }
    }

    public void addShuttlingToRequestedState() {
        if(requestedState == MacroState.IDLE) {
            requestedState = MacroState.SHUTTLING;
        }
        else if(requestedState == MacroState.INTAKING) {
            requestedState = MacroState.STEADY_STREAM_SHUTTLING;
        }
        else if(requestedState == MacroState.SCORING) {
            requestedState = MacroState.SHUTTLING;
        }
        else if(requestedState == MacroState.STEADY_STREAM_SCORING) {
            requestedState = MacroState.STEADY_STREAM_SHUTTLING;
        }
    }

    public void requestShuttling() {
        addShuttlingToRequestedState();
    }

    public Command requestShuttlingCommand() {
        return runOnce(this::requestShuttling).withName("RequestShuttling");
    }

    public void turnOffLaunchingStates() {
        removeShuttlingFromRequestedState();
        removeScoringFromRequestedState();
    }

    public Command turnOffLaunchingStatesCommand() {
        return runOnce(this::turnOffLaunchingStates).withName("TurnOffLaunchingStates");
    }

    // ==================== Trigger Factory Methods ====================

    /**
     * Creates a Trigger that is true when the current state matches the given state.
     * @param state The MacroState to check against.
     * @return A Trigger that is true when currentState == state.
     */
    public static Trigger whenCurrentState(MacroState state) {
        return new Trigger(() -> currentState == state);
    }

    /**
     * Creates a Trigger that is true when the desired state matches the given state.
     * @param state The MacroState to check against.
     * @return A Trigger that is true when desiredState == state.
     */
    public static Trigger whenDesiredState(MacroState state) {
        return new Trigger(() -> requestedState == state);
    }

    /**
     * Creates a Trigger that is true when a state has a specific status.
     * @param state The MacroState to check.
     * @param status The Status to check for.
     * @return A Trigger that is true when state.getStatus() == status.
     */
    public static Trigger whenStateHasStatus(MacroState state, Status status) {
        return new Trigger(() -> state.getStatus() == status);
    }

    /**
     * Creates a Trigger that is true when the current state matches and has READY status.
     * Useful for triggering actions only when a state is fully prepared.
     * @param state The MacroState to check.
     * @return A Trigger that is true when currentState == state AND status == READY.
     */
    public static Trigger whenCurrentStateReady(MacroState state) {
        return new Trigger(() -> currentState == state && state.getStatus() == Status.READY);
    }

    /**
     * Creates a Trigger that is true when the current state matches and has INITIALIZING status.
     * Useful for triggering spin-up or preparation actions.
     * @param state The MacroState to check.
     * @return A Trigger that is true when currentState == state AND status == INITIALIZING.
     */
    public static Trigger whenCurrentStateInitializing(MacroState state) {
        return new Trigger(() -> currentState == state && state.getStatus() == Status.WAITING);
    }

    /**
     * Creates a Trigger that is true when transitioning away from a state (status == STOPPING).
     * Useful for cleanup or safe shutdown actions.
     * @param state The MacroState to check.
     * @return A Trigger that is true when state.getStatus() == STOPPING.
     */
    public static Trigger whenStateStopping(MacroState state) {
        return new Trigger(() -> state.getStatus() == Status.STOPPING);
    }

    public static Trigger whenIntakeNotCurrent() {
        return new Trigger(
            () -> currentState != MacroState.INTAKING && currentState != MacroState.STEADY_STREAM_SCORING && currentState != MacroState.STEADY_STREAM_SHUTTLING
        );
    }

    public static Trigger whenIntakeNotDesired() {
        return new Trigger(
            () -> requestedState != MacroState.INTAKING && requestedState != MacroState.STEADY_STREAM_SCORING && requestedState != MacroState.STEADY_STREAM_SHUTTLING
        );
    }

    @Override
    public void addPathPlannerCommands() {
        PathPlannerCommands.addCommand("Request Idle State", requestStateCommand(MacroState.IDLE));
        PathPlannerCommands.addCommand("Request Scoring State", requestStateCommand(MacroState.SCORING));
        PathPlannerCommands.addCommand("Request Shuttling State", requestStateCommand(MacroState.SHUTTLING));
        PathPlannerCommands.addCommand("Request Intaking State", requestStateCommand(MacroState.INTAKING));
        PathPlannerCommands.addCommand("Request Climbing State", requestStateCommand(MacroState.CLIMBING));
        PathPlannerCommands.addCommand("Request SS Scoring State", requestStateCommand(MacroState.STEADY_STREAM_SCORING));
        PathPlannerCommands.addCommand("Request SS Shuttling State", requestStateCommand(MacroState.STEADY_STREAM_SHUTTLING));

        PathPlannerCommands.addCommand("Force Idle State", setCurrentStateCommand(MacroState.IDLE));
        PathPlannerCommands.addCommand("Force Scoring State", setCurrentStateCommand(MacroState.SCORING));
        PathPlannerCommands.addCommand("Force Shuttling State", setCurrentStateCommand(MacroState.SHUTTLING));
        PathPlannerCommands.addCommand("Force Intaking State", setCurrentStateCommand(MacroState.INTAKING));
        PathPlannerCommands.addCommand("Force Climbing State", setCurrentStateCommand(MacroState.CLIMBING));
        PathPlannerCommands.addCommand("Force SS Scoring State", setCurrentStateCommand(MacroState.STEADY_STREAM_SCORING));
        PathPlannerCommands.addCommand("Force SS Shuttling State", setCurrentStateCommand(MacroState.STEADY_STREAM_SHUTTLING));
    }
}
