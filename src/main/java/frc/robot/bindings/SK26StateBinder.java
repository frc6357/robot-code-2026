package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.StateHandler;
import frc.robot.Ports.DriverPorts;
import frc.robot.Ports.OperatorPorts;
import frc.robot.StateHandler.MacroState;

public class SK26StateBinder implements CommandBinder {
    private Optional<StateHandler> stateHandlerContainer;
    private StateHandler stateHandler;

    Trigger turnOffLaunch;
    Trigger turnOnScoring;
    Trigger turnOnShuttling;

    // Trigger that is true when launcher is in an active launching state
    Trigger launcherActive;
    // Debouncer to ensure launcher has been active for 0.8 seconds before allowing turn off
    Debouncer launcherActiveDebouncer = new Debouncer(0.8, DebounceType.kRising);

    public SK26StateBinder(Optional<StateHandler> stateHandlerContainer) {
        this.stateHandlerContainer = stateHandlerContainer;
        this.stateHandler = stateHandlerContainer.orElse(null);

        turnOnScoring = DriverPorts.kRTrigger.button;
        turnOnShuttling = DriverPorts.kLeftDpad.button;

        if (stateHandler != null) {
            // Create trigger for when launcher is in any active launching state
            launcherActive = new Trigger(() -> {
                MacroState state = stateHandler.getCurrentState();
                return state == MacroState.SCORING 
                    || state == MacroState.SHUTTLING
                    || state == MacroState.STEADY_STREAM_SCORING 
                    || state == MacroState.STEADY_STREAM_SHUTTLING;
            });

            // Turn off trigger requires: double-press RT AND launcher has been active for 0.8s
            Trigger launcherActiveFor800ms = new Trigger(() -> launcherActiveDebouncer.calculate(launcherActive.getAsBoolean()));
            turnOffLaunch = DriverPorts.kRTrigger.button.multiPress(2, 0.33).and(launcherActiveFor800ms);
        } else {
            turnOffLaunch = DriverPorts.kRTrigger.button.multiPress(2, 0.33);
        }
    }

    @Override
    public void bindButtons() {
        if(stateHandlerContainer.isEmpty()) {
            return;
        }
        bindDriverButtons();
        bindOperatorButtons();
    }
    
    private void bindDriverButtons() 
    {
        DriverPorts.kLTrigger.button.onTrue(stateHandler.toggleIntakeInRequestedStateCommand());

        turnOnScoring.onTrue(stateHandler.requestScoringCommand());
        turnOnShuttling.onTrue(stateHandler.requestShuttlingCommand());
        turnOffLaunch.onTrue(stateHandler.turnOffLaunchingStatesCommand());
    }

    private void bindOperatorButtons() {
        OperatorPorts.kRTrigger.button.onTrue(stateHandler.requestStateCommand(MacroState.SCORING));
        OperatorPorts.kLTrigger.button.onTrue(stateHandler.requestStateCommand(MacroState.SHUTTLING));
        OperatorPorts.kStartbutton.button.onTrue(stateHandler.setCurrentStateCommand(MacroState.IDLE));
    }
}
