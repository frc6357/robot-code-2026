package frc.robot.bindings;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Konstants.LauncherConstants.kStationaryThresholdMetersPerSecond;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.subsystems.launcher.moveandshoot.ShootingCoordinator;

public class SK26ShootingCoordinatorBinder implements CommandBinder {
    private Optional<ShootingCoordinator> moveAndShootSystemContainer;

    Trigger score = StateHandler.whenCurrentState(MacroState.SCORING).or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING));
    Trigger shuttle = StateHandler.whenCurrentState(MacroState.SHUTTLING).or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING));
    Trigger stationary;
    Trigger moving;

    public SK26ShootingCoordinatorBinder(Optional<ShootingCoordinator> moveAndShootSystemContainer) {
        this.moveAndShootSystemContainer = moveAndShootSystemContainer;
    }

    @Override
    public void bindButtons() {
        if (moveAndShootSystemContainer.isEmpty()) {
            return;
        }

        ShootingCoordinator moveAndShootSystem = moveAndShootSystemContainer.get();

        // Bind triggers for stationary vs moving shots based on robot velocity
        stationary = new Trigger(() -> {
            double vx = moveAndShootSystem.getDrive().getVelocity(true).vxMetersPerSecond;
            double vy = moveAndShootSystem.getDrive().getVelocity(true).vyMetersPerSecond;
            return Math.hypot(vx, vy) < kStationaryThresholdMetersPerSecond.in(MetersPerSecond);  // Consider stationary if speed is less than threshold
        });
        moving = stationary.negate();

        // When score or shuttle triggers are active, run the appropriate shooting command based on whether we're stationary or moving
        //TODO: Create commands that choreograph the appropriate shooting sequence for stationary vs moving shots, add them into Trigger bindings
        // score.and(stationary).whileTrue();
        // score.and(moving).whileTrue();

        // shuttle.and(stationary).whileTrue();
        // shuttle.and(moving).whileTrue();

    }
}