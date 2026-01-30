package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.commands.RunLauncherWithRPSCommand;
// import frc.robot.commands.RunLauncherWithVelCommand;
// import frc.robot.commands.commandGroups.LauncherUnJamCommandGroup;
// import frc.robot.subsystems.SK26Launcher;
import frc.robot.subsystems.launcher.BangBangLauncher;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Konstants.LauncherConstants.kTargetMotorRPS;
import static frc.robot.Konstants.LauncherConstants.kTargetlaunchVelocity;
import static frc.robot.Ports.OperatorPorts.kIntake;
import static frc.robot.Ports.OperatorPorts.kShootExitVel;
import static frc.robot.Ports.OperatorPorts.kShootRPS;
import static frc.robot.Ports.OperatorPorts.kUnJam;

public class SK26LauncherBinder implements CommandBinder {

    Optional<BangBangLauncher> launcherSubsystem;

    Trigger ShootExitVel;
    Trigger ShootRPS;
    Trigger UnJam;
    
    public SK26LauncherBinder(Optional<BangBangLauncher> launcherSubsystem) {
        this.launcherSubsystem = launcherSubsystem;
        this.ShootExitVel = kShootExitVel.button;
        this.ShootRPS = kIntake.button;
        this.UnJam = kUnJam.button;
    }

    @Override
    public void bindButtons() {
        
        if(launcherSubsystem.isPresent()) {

            // SK26Launcher launcher = launcherSubsystem.get();
            BangBangLauncher launcher = launcherSubsystem.get();

            // if(ShootExitVel.getAsBoolean() && UnJam.getAsBoolean()) {
            //     ShootExitVel.whileTrue(new RunLauncherWithVelCommand(launcher, kTargetlaunchVelocity));
            // } else if(ShootRPS.getAsBoolean() && UnJam.getAsBoolean()) {
            //     ShootRPS.whileTrue(new RunLauncherWithRPSCommand(launcher, kTargetMotorRPS));
            // } else {
            //     ShootExitVel.whileTrue(new RunLauncherWithVelCommand(launcher, kTargetlaunchVelocity));
            //     ShootRPS.whileTrue(new RunLauncherWithRPSCommand(launcher, kTargetMotorRPS));
            //     UnJam.whileTrue(new LauncherUnJamCommandGroup(launcher));
            // }

            ShootRPS.whileTrue(launcher.runFixedSpeedCommand(() -> RotationsPerSecond.of(24.5 / 2)));
        }
    }
    
}
