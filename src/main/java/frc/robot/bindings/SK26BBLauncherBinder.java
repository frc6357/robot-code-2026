package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;
// import frc.robot.commands.RunLauncherWithRPSCommand;
// import frc.robot.commands.RunLauncherWithVelCommand;
// import frc.robot.commands.commandGroups.LauncherUnJamCommandGroup;
// import frc.robot.subsystems.SK26Launcher;
import frc.robot.subsystems.launcher.BangBangLauncher;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Konstants.LauncherConstants.kTargetMotorRPS;
import static frc.robot.Konstants.LauncherConstants.kTargetlaunchVelocity;
import static frc.robot.Ports.OperatorPorts.kRTrigger;
import static frc.robot.Ports.OperatorPorts.kLTrigger;
import static frc.robot.Ports.OperatorPorts.kLBbutton;
import static frc.robot.Ports.OperatorPorts.kXbutton;

public class SK26BBLauncherBinder implements CommandBinder {

    Optional<BangBangLauncher> launcherSubsystem;

    Trigger ShootExitVel;
    Trigger ShootRPS;
    Trigger UnJam;

    Pref<Double> kShootVelocity = SKPreferences.attach("BBLauncher/ManualShootVelocity (rps)", 24.5);
    
    public SK26BBLauncherBinder(Optional<BangBangLauncher> launcherSubsystem) {
        this.launcherSubsystem = launcherSubsystem;
        this.ShootExitVel = kLTrigger.button;
        this.ShootRPS = kRTrigger.button;
        this.UnJam = kXbutton.button;
    }

    @Override
    public void bindButtons() {
        
        if(launcherSubsystem.isPresent()) {
            BangBangLauncher launcher = launcherSubsystem.get();

            ShootRPS.whileTrue(launcher.runFixedSpeedCommand(() -> RotationsPerSecond.of(kShootVelocity.get() / 2)));
        }
    }
    
}