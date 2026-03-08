package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;
import frc.robot.commands.RunLauncherWithRPSCommand;
import frc.robot.commands.RunLauncherWithVelCommand;
import frc.robot.commands.commandGroups.LauncherUnJamCommandGroup;
import frc.robot.subsystems.launcher.mechanisms.SK26Launcher;

import static frc.robot.Konstants.LauncherConstants.kTargetMotorRPS;
import static frc.robot.Konstants.LauncherConstants.kTargetlaunchVelocity;
import static frc.robot.Ports.OperatorPorts.kRTrigger;
import static frc.robot.Ports.OperatorPorts.kLTrigger;
import static frc.robot.Ports.OperatorPorts.kXbutton;

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

            ShootExitVel.whileTrue(new RunLauncherWithVelCommand(launcher, kTargetlaunchVelocity));
            ShootRPS.whileTrue(new RunLauncherWithRPSCommand(launcher, kTargetMotorRPS));
            UnJam.whileTrue(new LauncherUnJamCommandGroup(launcher));
        }
    }
    
}
