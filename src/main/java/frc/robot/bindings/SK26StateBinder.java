package frc.robot.bindings;

import frc.robot.StateHandler;
import frc.robot.Ports.DriverPorts;
import frc.robot.Ports.OperatorPorts;
import frc.robot.StateHandler.MacroState;

public class SK26StateBinder implements CommandBinder {
    private StateHandler stateHandler;

    public SK26StateBinder(StateHandler stateHandler) {
        this.stateHandler = stateHandler;
    }

    @Override
    public void bindButtons() {
        bindDriverButtons();
        bindOperatorButtons();
    }
    
    private void bindDriverButtons() {
        DriverPorts.kRTrigger.button.onTrue(stateHandler.setDesiredStateCommand(MacroState.CLIMBING));
    }

    private void bindOperatorButtons() {
        OperatorPorts.kRTrigger.button.onTrue(stateHandler.setDesiredStateCommand(MacroState.LAUNCHING));
        OperatorPorts.kLTrigger.button.onTrue(stateHandler.setDesiredStateCommand(MacroState.INTAKING));

    }
}
