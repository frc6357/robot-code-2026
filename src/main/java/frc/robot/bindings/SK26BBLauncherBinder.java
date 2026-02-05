package frc.robot.bindings;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Ports.OperatorPorts.kLTrigger;
import static frc.robot.Ports.OperatorPorts.kRTrigger;
import static frc.robot.Ports.OperatorPorts.kXbutton;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.subsystems.launcher.BangBangLauncher;

public class SK26BBLauncherBinder implements CommandBinder {

    Optional<BangBangLauncher> launcherSubsystem;

    Trigger ShootExitVel;
    Trigger ShootRPS;
    Trigger UnJam;
    Trigger Shoot;

    Pref<Double> kShootVelocity = SKPreferences.attach("BBLauncher/ManualShootVelocity (rps)", 24.5);
    
    public SK26BBLauncherBinder(Optional<BangBangLauncher> launcherSubsystem) {
        this.launcherSubsystem = launcherSubsystem;
        this.ShootExitVel = kLTrigger.button;
        this.ShootRPS = kRTrigger.button.and(StateHandler.whenCurrentState(MacroState.IDLE));
        this.UnJam = kXbutton.button;
        Shoot = StateHandler.whenCurrentState(MacroState.SCORING)
                .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING))
                .or(StateHandler.whenCurrentState(MacroState.SHUTTLING))
                .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING));
    }

    @Override
    public void bindButtons() {
        
        if(launcherSubsystem.isPresent()) {
            BangBangLauncher launcher = launcherSubsystem.get();

            ShootRPS.whileTrue(launcher.runFixedSpeedCommand(() -> RotationsPerSecond.of(kShootVelocity.get() / 2)));

            //TODO: This will need to be changed to a velocity control command when we implement velocity control
            Shoot.whileTrue(launcher.runFixedSpeedCommand(() -> RotationsPerSecond.of(kShootVelocity.get() / 2)));
        }
    }
    
}