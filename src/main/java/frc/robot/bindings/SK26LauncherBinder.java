package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RunLauncherCommand;
import frc.robot.subsystems.SK26Launcher;
import static frc.robot.Ports.OperatorPorts.kShoot;

public class SK26LauncherBinder implements CommandBinder {

    Optional<SK26Launcher> launcherSubsystem;

    Trigger Shoot;
    
    public SK26LauncherBinder(Optional<SK26Launcher> launcherSubsystem) {
        this.launcherSubsystem = launcherSubsystem;
        this.Shoot = kShoot.button;
    }

    @Override
    public void bindButtons() {
        
        if(launcherSubsystem.isPresent()) {

            SK26Launcher launcher = launcherSubsystem.get();

            Shoot.whileTrue(new RunLauncherCommand(launcher));
        }
    }
    
}
