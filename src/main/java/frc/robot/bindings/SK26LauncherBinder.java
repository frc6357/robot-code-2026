package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RunLauncherWithRPSCommand;
import frc.robot.commands.RunLauncherWithVelCommand;
import frc.robot.commands.commandGroups.LauncherUnJamCommandGroup;
import frc.robot.subsystems.SK26Launcher;

import static frc.robot.Konstants.LauncherConstants.kTargetMotorRPS;
import static frc.robot.Konstants.LauncherConstants.kTargetlaunchVelocity;
import static frc.robot.Ports.OperatorPorts.kShootExitVel;
import static frc.robot.Ports.OperatorPorts.kShootRPS;
import static frc.robot.Ports.OperatorPorts.kUnJam;

public class SK26LauncherBinder implements CommandBinder {

    Optional<SK26Launcher> launcherSubsystem;

    Trigger ShootExitVel;
    Trigger ShootRPS;
    Trigger UnJam;
    
    public SK26LauncherBinder(Optional<SK26Launcher> launcherSubsystem) {
        this.launcherSubsystem = launcherSubsystem;
        this.ShootExitVel = kShootExitVel.button;
        this.ShootRPS = kShootRPS.button;
        this.UnJam = kUnJam.button;
    }

    @Override
    public void bindButtons() {
        
        if(launcherSubsystem.isPresent()) {

            SK26Launcher launcher = launcherSubsystem.get();

            ShootExitVel.whileTrue(new RunLauncherWithVelCommand(launcher, kTargetlaunchVelocity));
            ShootRPS.whileTrue(new RunLauncherWithRPSCommand(launcher, kTargetMotorRPS));
            UnJam.and(ShootRPS.negate()).and(ShootExitVel.negate())
                .whileTrue(new LauncherUnJamCommandGroup(launcher));
        }
    }
    
}
