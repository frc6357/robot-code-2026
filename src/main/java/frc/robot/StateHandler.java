package frc.robot;
    
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.StateHandler.MacroState.Status;

/**
 * A class to handle large-scale robot states (macros) such as launching, intaking, climbing, and idling.
 * Each macro state has an associated status to indicate its current condition.
 * A MacroState can still have its status updated while it is not the current nor desired state.
 */
public class StateHandler extends SubsystemBase{
    
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
            INITIALIZING,
            READY,
            STOPPING
        }
    }
    private SendableChooser<MacroState> stateChooser = new SendableChooser<MacroState>();        

    private static MacroState currentState = MacroState.IDLE;
    private static MacroState desiredState = MacroState.IDLE;

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
        if(desiredState != currentState) {
            currentState.setStatus(Status.STOPPING);
            currentState = desiredState;
            currentState.setStatus(Status.INITIALIZING);
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
        builder.addStringProperty("Desired State", () -> getDesiredState().name(), null);

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
        clearDesiredState();
    }

    public Command setCurrentStateCommand(MacroState state) {
        return runOnce(() -> setCurrentState(state));
    }

    /**
     * Gets the desired state.
     * @return The desired MacroState.
     */
    public MacroState getDesiredState() {
        return desiredState;
    }

    /**
     * Sets the desired state.
     * @param state The desired MacroState.
     */
    public void setDesiredState(MacroState state) {
        desiredState = state;
    }

    public Command setDesiredStateCommand(MacroState state) {
        return runOnce(() -> setDesiredState(state));
    }

    /**
     * Clears the desired MacroState, setting it to IDLE.
     */
    public void clearDesiredState() {
        desiredState = currentState;
    }

    public Command clearDesiredStateCommand() {
        return runOnce(this::clearDesiredState);
    }

    public MacroState.Status getStatusOf(MacroState state) {
        return state.getStatus();
    }
    
    public void setStatusOf(MacroState state, MacroState.Status status) {
        state.setStatus(status);
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
        return new Trigger(() -> desiredState == state);
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
        return new Trigger(() -> currentState == state && state.getStatus() == Status.INITIALIZING);
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
}
