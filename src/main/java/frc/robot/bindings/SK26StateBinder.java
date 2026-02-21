package frc.robot.bindings;

import java.util.Optional;

import frc.robot.StateHandler;
import frc.robot.Ports.DriverPorts;
import frc.robot.Ports.OperatorPorts;
import frc.robot.StateHandler.MacroState;

public class SK26StateBinder implements CommandBinder {
    private Optional<StateHandler> stateHandlerContainer;
    private StateHandler stateHandler;

    public SK26StateBinder(Optional<StateHandler> stateHandlerContainer) {
        this.stateHandlerContainer = stateHandlerContainer;
        this.stateHandler = stateHandlerContainer.orElse(null);
    }

    @Override
    public void bindButtons() {
        if(stateHandlerContainer.isEmpty()) {
            return;
        }
        bindDriverButtons();
        bindOperatorButtons();
    }
    
    private void bindDriverButtons() {
        DriverPorts.kLTrigger.button.onTrue(stateHandler.toggleIntakeInRequestedStateCommand());

        DriverPorts.kRTrigger.button.onTrue(stateHandler.requestScoringCommand());
        DriverPorts.kRTrigger.button.debounce(1).onTrue(stateHandler.requestShuttlingCommand());
        DriverPorts.kRTrigger.button.multiPress(2, 0.8).onTrue(stateHandler.turnOffLaunchingStatesCommand());
    }

    private void bindOperatorButtons() {
        OperatorPorts.kRTrigger.button.onTrue(stateHandler.requestStateCommand(MacroState.SCORING));
        OperatorPorts.kLTrigger.button.onTrue(stateHandler.requestStateCommand(MacroState.SHUTTLING));

        OperatorPorts.kStartbutton.button.onTrue(stateHandler.setCurrentStateCommand(MacroState.IDLE));
    }
}
