package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kLeftStickY;
import static frc.robot.Ports.OperatorPorts.kLeftStickX;
import frc.lib.utils.filters.LinearDeadbandFilter;
import frc.lib.utils.Field;
import static frc.robot.Konstants.TargetPointConstants.TargetPoint.kOperatorControlled;


import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SKTargetPointsBinder implements CommandBinder {
    public SKTargetPointsBinder() {
        // Initialize target point at the origin (0,0); could be changed to a different default location
        kLeftStickY.setFilter(new LinearDeadbandFilter(0.15, 1.0));
        kLeftStickX.setFilter(new LinearDeadbandFilter(0.15, 1.0));
    }

    @Override
    public void bindButtons() {
        kOperatorControlled.point.setDefaultCommand(
            new InstantCommand(
                () -> kOperatorControlled.point.moveTargetPoint(
                    (Field.isBlue() ? -1 : 1) * 0.1 * kLeftStickY.getFilteredAxis(), 
                    (Field.isBlue() ? -1 : 1) * 0.1 * kLeftStickX.getFilteredAxis()), 
                    kOperatorControlled.point).ignoringDisable(true).withName("OperatorTargetPointMover")
        );
    }
    
}
