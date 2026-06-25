package frc.robot.bindings;

import java.lang.StackWalker.Option;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.subsystems.launcher.mechanisms.SK26DualLauncher;
import frc.robot.subsystems.launcher.moveandshoot.ShootingCoordinator;

import static frc.robot.Ports.OperatorPorts.kLBbutton;

public class SK26ShootingCoordinatorBinder implements CommandBinder {
    private Optional<ShootingCoordinator> moveAndShootSystemContainer;
    private Optional<SK26DualLauncher> launcherSubsystem;

    Trigger score = StateHandler.whenCurrentState(MacroState.SCORING).or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING)).or(StateHandler.whenCurrentState(MacroState.CLIMB_AND_SCORE));
    Trigger shuttle = StateHandler.whenCurrentState(MacroState.SHUTTLING).or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING));
    Trigger stationary;
    Trigger moving;
    Trigger idle = StateHandler.whenCurrentState(MacroState.IDLE);
    Trigger manualShoot = kLBbutton.button;

    public SK26ShootingCoordinatorBinder(Optional<ShootingCoordinator> moveAndShootSystemContainer, Optional<SK26DualLauncher> launcherSubsystem) {
        this.moveAndShootSystemContainer = moveAndShootSystemContainer;
        this.launcherSubsystem = launcherSubsystem;
    }

    @Override
    public void bindButtons() {
        if (moveAndShootSystemContainer.isEmpty()) {
            return;
        }

        ShootingCoordinator moveAndShootSystem = moveAndShootSystemContainer.get();
        SK26DualLauncher launcher = launcherSubsystem.get();

        // Bind triggers for stationary vs moving shots based on robot velocity
        // stationary = new Trigger(() -> {
        //     double vx = moveAndShootSystem.getDrive().getVelocity(true).vxMetersPerSecond;
        //     double vy = moveAndShootSystem.getDrive().getVelocity(true).vyMetersPerSecond;
        //     return Math.hypot(vx, vy) < kStationaryThresholdMetersPerSecond.in(MetersPerSecond);  // Consider stationary if speed is less than threshold
        // });
        // moving = stationary.negate();

        // When score or shuttle triggers are active, run the appropriate shooting command
        score.and(manualShoot).whileTrue(
            launcher.runVelocityFromPrefCommand()
        );
        score.and(manualShoot.negate()).whileTrue(moveAndShootSystem.scoreMoving());
        shuttle.whileTrue(moveAndShootSystem.shuttleMoving());
    }
}