package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;
import frc.robot.subsystems.launcher.mechanisms.SK26Launcher;

import static frc.robot.Konstants.LauncherConstants.kTargetMotorRPS;
import static frc.robot.Konstants.LauncherConstants.kTargetlaunchVelocity;
import static frc.robot.Ports.OperatorPorts.kRTrigger;
import static frc.robot.Ports.OperatorPorts.kLTrigger;
import static frc.robot.Ports.OperatorPorts.kXbutton;

import static frc.robot.Ports.TesterPorts.kLauncherButton;

public class SK26LauncherBinder implements CommandBinder {

    Optional<SK26Launcher> launcherSubsystem;

    Trigger ShootExitVel;
    Trigger ShootRPS;
    Trigger UnJam;

    Pref<Double> kShootVelocity = SKPreferences.attach("BBLauncher/ManualShootVelocity (rps)", 24.5);
    
    public SK26LauncherBinder(Optional<SK26Launcher> launcherSubsystem) {
        this.launcherSubsystem = launcherSubsystem;
        this.ShootExitVel = kLTrigger.button;
        this.ShootRPS = kRTrigger.button;
        this.UnJam = kXbutton.button;
    }

    @Override
    public void bindButtons() {
        
        if(launcherSubsystem.isPresent()) {

            // SK26Launcher launcher = launcherSubsystem.get();
            SK26Launcher launcher = launcherSubsystem.get();

            ShootExitVel.whileTrue(launcher.runAtExitVelCommand(kTargetlaunchVelocity));
            ShootRPS.whileTrue(launcher.runAtRPSCommand(kTargetMotorRPS));
            UnJam.whileTrue(launcher.unjamCommand());

            kLauncherButton.button.whileTrue(launcher.runAtExitVelCommand(kTargetlaunchVelocity));
        }
    }
    
}
