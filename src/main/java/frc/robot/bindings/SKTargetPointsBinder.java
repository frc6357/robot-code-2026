package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kLeftStickY;
import static frc.robot.Ports.OperatorPorts.kLeftStickX;
import frc.lib.utils.filters.LinearDeadbandFilter;
import frc.lib.utils.Field;
import static frc.robot.Konstants.TargetPointConstants.TargetPoint.kOperatorControlled;
import static frc.robot.Konstants.TargetPointConstants.targetPoints;


import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SKTargetPointsBinder implements CommandBinder {
    public SKTargetPointsBinder() {
        // Initialize target point at the origin (0,0); could be changed to a different default location
        kLeftStickY.setFilter(new LinearDeadbandFilter(0.15, 1.0));
        kLeftStickX.setFilter(new LinearDeadbandFilter(0.15, 1.0));
    }

    @Override
    public void bindButtons() {
        targetPoints[kOperatorControlled.ordinal()].setDefaultCommand(
            new InstantCommand(
                () -> targetPoints[kOperatorControlled.ordinal()].moveTargetPoint(
                    (Field.isBlue() ? -1 : 1) * 0.1 * kLeftStickY.getFilteredAxis(), 
                    (Field.isBlue() ? -1 : 1) * 0.1 * kLeftStickX.getFilteredAxis()), 
                    targetPoints[kOperatorControlled.ordinal()]).ignoringDisable(true)
        );
    }
    
}
