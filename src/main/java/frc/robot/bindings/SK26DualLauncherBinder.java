package frc.robot.bindings;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Konstants.TargetPointConstants.TargetPoint.kOperatorControlled;
import static frc.robot.Ports.OperatorPorts.kLBbutton;
import static frc.robot.Ports.OperatorPorts.kRTrigger;

import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.lib.utils.Field;
import frc.lib.utils.FieldConstants;
import frc.robot.Konstants.LauncherConstants;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.launcher.mechanisms.SK26DualLauncher;

public class SK26DualLauncherBinder implements CommandBinder {

    Optional<SK26DualLauncher> launcherSubsystem;
    Optional<SKSwerve> drive;

    Trigger ManualShoot;
    Trigger Score;
    Trigger TuningRun;
    Trigger Shuttle;

    InterpolatingDoubleTreeMap flywheelMap = LauncherConstants.createFlywheelSpeedMap();

    public SK26DualLauncherBinder(Optional<SK26DualLauncher> launcherSubsystem, Optional<SKSwerve> drive) {
        this.launcherSubsystem = launcherSubsystem;
        this.drive = drive;

        ManualShoot = kRTrigger.button/*.and(StateHandler.whenCurrentState(MacroState.IDLE))*/;

        TuningRun = kLBbutton.button.and(StateHandler.whenCurrentState(MacroState.IDLE));

        Score = StateHandler.whenCurrentState(MacroState.SCORING)
                .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING));
        
        Shuttle = StateHandler.whenCurrentState(MacroState.SHUTTLING)
                .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING));
    }

    @Override
    public void bindButtons() {
        if (launcherSubsystem.isPresent()) {
            SK26DualLauncher launcher = launcherSubsystem.get();

            if(drive.isEmpty()) {
                Score.whileTrue(
                    // Commands.defer(() -> launcher.runVelocityCommand(() -> RotationsPerSecond.of(kManualShootVelocity.get())), Set.of(launcher)))
                    Commands.defer(() -> launcher.runVelocityFromPrefCommand(), Set.of(launcher))
                );
            }
            else {                
                Score.whileTrue(
                    launcher.runVelocityCommand(
                        () -> RPM.of(flywheelMap.get(
                            drive.get().getRobotPose().getTranslation().getDistance(
                                Field.isBlue() ? FieldConstants.Hub.topCenterPoint.toTranslation2d() :
                                                FieldConstants.Hub.redTopCenterPoint.toTranslation2d()
                            )))
                    ).withName("LauncherScoreInterp")
                );

                Shuttle.whileTrue(
                    launcher.runVelocityCommand(
                        () -> RPM.of(flywheelMap.get(
                            drive.get().getRobotPose().getTranslation().getDistance(
                                kOperatorControlled.point.getTargetPoint()
                            )))
                    ).withName("LauncherShuttleInterp")
                );

                // Score.whileTrue(
                //     Commands.defer(() -> launcher.runVelocityFromPrefCommand(), Set.of(launcher))
                // );
                // Shuttle.whileTrue(
                //     Commands.defer(() -> launcher.runVelocityFromPrefCommand(), Set.of(launcher))
                // );
            }

            // PID tuning: hold operator LB (in IDLE) to spin at dashboard setpoint.
            // Adjust gains and setpoint live on SmartDashboard, watch response in AdvantageScope.
            // TuningRun.toggleOnTrue(launcher.tuningCommand());

            // Shoot trigger is available for ShootingCoordinator integration
            // Shoot.whileTrue(launcher.runVelocityCommand(() -> RotationsPerSecond.of(kManualShootVelocity.get())));
        }
    }
}
