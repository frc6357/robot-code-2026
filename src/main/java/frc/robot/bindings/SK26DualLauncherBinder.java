package frc.robot.bindings;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Ports.OperatorPorts.kRTrigger;
import static frc.robot.Konstants.TargetPointConstants.TargetPoint.kOperatorControlled;
import static frc.robot.Ports.OperatorPorts.kLBbutton;

import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;
import frc.robot.StateHandler;
import frc.robot.Konstants.LauncherConstants;
import frc.robot.StateHandler.MacroState;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.launcher.mechanisms.SK26DualLauncher;

public class SK26DualLauncherBinder implements CommandBinder {

    Optional<SK26DualLauncher> launcherSubsystem;
    Optional<SKSwerve> drive;

    Trigger ManualShoot;
    Trigger Shoot;
    Trigger TuningRun;

    InterpolatingDoubleTreeMap flywheelMap = LauncherConstants.createFlywheelSpeedMap();

    Pref<Double> kManualShootVelocity = SKPreferences.attach("DualLauncher/ManualShootVelocity (rps)", 35.0);

    public SK26DualLauncherBinder(Optional<SK26DualLauncher> launcherSubsystem, Optional<SKSwerve> drive) {
        this.launcherSubsystem = launcherSubsystem;
        this.drive = drive;

        ManualShoot = kRTrigger.button/*.and(StateHandler.whenCurrentState(MacroState.IDLE))*/;

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

            if(drive.isEmpty()) {
                ManualShoot.whileTrue(
                    // Commands.defer(() -> launcher.runVelocityCommand(() -> RotationsPerSecond.of(kManualShootVelocity.get())), Set.of(launcher)))
                    Commands.defer(() -> launcher.runVelocityFromPrefCommand(), Set.of(launcher))
                );
            }
            else {                
                ManualShoot.whileTrue(
                    launcher.runVelocityCommand(
                        () -> RPM.of(flywheelMap.get(
                            drive.get().getRobotPose().getTranslation().getDistance(kOperatorControlled.point.getTargetPoint())))
                    )
                );

                Shoot.whileTrue(
                    launcher.runVelocityCommand(
                        () -> RPM.of(flywheelMap.get(
                            drive.get().getRobotPose().getTranslation().getDistance(kOperatorControlled.point.getTargetPoint())))
                    )
                );

                // Shoot.whileTrue(
                //     Commands.defer(() -> launcher.runVelocityFromPrefCommand(), Set.of(launcher))
                // );
            }

            // PID tuning: hold operator LB (in IDLE) to spin at dashboard setpoint.
            // Adjust gains and setpoint live on SmartDashboard, watch response in AdvantageScope.
            TuningRun.toggleOnTrue(launcher.tuningCommand());

            // Shoot trigger is available for ShootingCoordinator integration
            // Shoot.whileTrue(launcher.runVelocityCommand(() -> RotationsPerSecond.of(kManualShootVelocity.get())));
        }
    }
}
