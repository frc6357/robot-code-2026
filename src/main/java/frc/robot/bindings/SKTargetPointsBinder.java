package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kLeftStickY;

import static frc.robot.Ports.OperatorPorts.kLeftStickX;
import frc.lib.utils.filters.LinearDeadbandFilter;
import frc.lib.bindings.CommandBinder;
import frc.lib.utils.Field;
import static frc.robot.Konstants.TargetPointConstants.TargetPoint.kOperatorControlled;


public class SKTargetPointsBinder implements CommandBinder {
    public SKTargetPointsBinder() {
        // Initialize target point at the origin (0,0); could be changed to a different default location
        kLeftStickY.setFilter(new LinearDeadbandFilter(0.15, 1.0));
        kLeftStickX.setFilter(new LinearDeadbandFilter(0.15, 1.0));
    }

    @Override
    public void bindButtons() {
        kOperatorControlled.point.setDefaultCommand(
            kOperatorControlled.point.movePointCommand(
                    () -> (Field.isBlue() ? -1 : 1) * 0.1 * kLeftStickY.getFilteredAxis(), 
                    () -> (Field.isBlue() ? -1 : 1) * 0.1 * kLeftStickX.getFilteredAxis())
                    .ignoringDisable(true).withName("OperatorTargetPointMover")
        );

        // kOperatorControlled.point.setDefaultCommand(
        //     kOperatorControlled.point.movePointCommand(
        //         (Field.isBlue() ? -1 : 1) * 0.1 * kLeftStickY.getFilteredAxis(), 
        //         (Field.isBlue() ? -1 : 1) * 0.1 * kLeftStickX.getFilteredAxis())
        //     .withName("OperatorTargetPointMover").ignoringDisable(true));
    }
    
}
