package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kMoveTargetX;
import static frc.robot.Ports.OperatorPorts.kMoveTargetY;
import frc.lib.utils.filters.LinearDeadbandFilter;
import frc.lib.utils.Field;
import static frc.robot.Konstants.TargetPointConstants.TargetPoint.kOperatorControlled;
import static frc.robot.Konstants.TargetPointConstants.targetPoints;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SKTargetPointsBinder implements CommandBinder {
    public SKTargetPointsBinder() {
        // Initialize target point at the origin (0,0); could be changed to a different default location
        kMoveTargetX.setFilter(new LinearDeadbandFilter(0.15, 1.0));
        kMoveTargetY.setFilter(new LinearDeadbandFilter(0.15, 1.0));
    }

    @Override
    public void bindButtons() {
        targetPoints[kOperatorControlled.ordinal()].setDefaultCommand(
            new InstantCommand(
                () -> targetPoints[kOperatorControlled.ordinal()].moveTargetPoint(
                    (Field.isBlue() ? -1 : 1) * 0.1 * kMoveTargetX.getFilteredAxis(), 
                    (Field.isBlue() ? -1 : 1) * 0.1 * kMoveTargetY.getFilteredAxis()), 
                    targetPoints[kOperatorControlled.ordinal()]).ignoringDisable(true)
        );
    }
    
}
