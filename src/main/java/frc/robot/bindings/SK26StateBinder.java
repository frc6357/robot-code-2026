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
        DriverPorts.kRTrigger.button.onTrue(stateHandler.requestStateCommand(MacroState.CLIMBING));
    }

    private void bindOperatorButtons() {
        OperatorPorts.kRTrigger.button.onTrue(stateHandler.requestStateCommand(MacroState.SCORING));
        OperatorPorts.kLTrigger.button.onTrue(stateHandler.requestStateCommand(MacroState.INTAKING));

        OperatorPorts.kStartbutton.button.onTrue(stateHandler.setCurrentStateCommand(MacroState.IDLE));
    }
}
