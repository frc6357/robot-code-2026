package frc.robot.bindings;

import frc.robot.subsystems.drive.SKTargetPoint;
import static frc.robot.Ports.OperatorPorts.kMoveTargetX;
import static frc.robot.Ports.OperatorPorts.kMoveTargetY;
import frc.lib.utils.filters.LinearDeadbandFilter;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SKTargetPointsBinder implements CommandBinder {
    SKTargetPoint operatorControlledTargetPoint;

    public SKTargetPointsBinder(Optional<SKTargetPoint> targetPoint) {
        // Initialize target point at the origin (0,0); could be changed to a different default location
        operatorControlledTargetPoint = targetPoint.get();

        kMoveTargetX.setFilter(new LinearDeadbandFilter(0.15, 1.0));
        kMoveTargetY.setFilter(new LinearDeadbandFilter(0.15, 1.0));
    }

    @Override
    public void bindButtons() {
        operatorControlledTargetPoint.setDefaultCommand(
            new InstantCommand(
                () -> operatorControlledTargetPoint.moveTargetPoint(
                    0.1 * -kMoveTargetX.getFilteredAxis(), 
                    0.1 * -kMoveTargetY.getFilteredAxis()), 
                    operatorControlledTargetPoint).ignoringDisable(true)
        );
    }
    
}
