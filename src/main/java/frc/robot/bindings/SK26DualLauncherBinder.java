package frc.robot.bindings;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Ports.OperatorPorts.kRTrigger;
import static frc.robot.Ports.OperatorPorts.kLBbutton;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.subsystems.launcher.mechanisms.SK26DualLauncher;

public class SK26DualLauncherBinder implements CommandBinder {

    Optional<SK26DualLauncher> launcherSubsystem;

    Trigger ManualShoot;
    Trigger Shoot;
    Trigger TuningRun;

    Pref<Double> kManualShootVelocity = SKPreferences.attach("DualLauncher/ManualShootVelocity (rps)", 40.0);

    public SK26DualLauncherBinder(Optional<SK26DualLauncher> launcherSubsystem) {
        this.launcherSubsystem = launcherSubsystem;

        ManualShoot = kRTrigger.button.and(StateHandler.whenCurrentState(MacroState.IDLE));

        TuningRun = kLBbutton.button.and(StateHandler.whenCurrentState(MacroState.IDLE));

        Shoot = StateHandler.whenCurrentState(MacroState.SCORING)
                .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING))
                .or(StateHandler.whenCurrentState(MacroState.SHUTTLING))
                .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING));
    }

    @Override
    public void bindButtons() {
        if (launcherSubsystem.isPresent()) {
            SK26DualLauncher launcher = launcherSubsystem.get();

            ManualShoot.whileTrue(
                launcher.runVelocityCommand(() -> RotationsPerSecond.of(kManualShootVelocity.get())));

            // PID tuning: hold operator LB (in IDLE) to spin at dashboard setpoint.
            // Adjust gains and setpoint live on SmartDashboard, watch response in AdvantageScope.
            TuningRun.toggleOnTrue(launcher.tuningCommand());

            // Shoot trigger is available for ShootingCoordinator integration
            // Shoot.whileTrue(launcher.runVelocityCommand(() -> RotationsPerSecond.of(kManualShootVelocity.get())));
        }
    }
}
