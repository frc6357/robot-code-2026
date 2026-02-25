package frc.robot.bindings;

// Imports from robot
import static frc.robot.Ports.OperatorPorts.kLTrigger;
import static frc.robot.Ports.OperatorPorts.kRTrigger;
import static frc.robot.Ports.OperatorPorts.kXbutton;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.subsystems.launcher.mechanisms.BangBangLauncher;

// Imports from Java/WPILib
import java.util.Optional;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SK26BBLauncherBinder implements CommandBinder {

    Optional<BangBangLauncher> launcherSubsystem;

    Trigger ShootExitVel;
    Trigger ShootRPS;
    Trigger UnJam;
    Trigger Shoot;

    Pref<Double> kShootVelocity = SKPreferences.attach("BBLauncher/ManualShootVelocity (rps)", 24.5);
    
    public SK26BBLauncherBinder(Optional<BangBangLauncher> launcherSubsystem) 
    {
        this.launcherSubsystem = launcherSubsystem;

        ShootExitVel = kLTrigger.button;
        UnJam = kXbutton.button;
        ShootRPS = kRTrigger.button.and(StateHandler.whenCurrentState(MacroState.IDLE));

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