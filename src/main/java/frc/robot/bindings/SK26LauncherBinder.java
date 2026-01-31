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

/**
 * Binds operator controls (Triggers) to launcher-related commands.
 *
 * <p>This binder wires up three actions:
 * <ul>
 *   <li><b>ShootExitVel</b>: while held, runs the launcher to a target exit velocity using
 *       {@link frc.robot.commands.RunLauncherWithVelCommand}.</li>
 *   <li><b>ShootRPS</b>: while held, runs the launcher to a target motor speed (RPS) using
 *       {@link frc.robot.commands.RunLauncherWithRPSCommand}.</li>
 *   <li><b>UnJam</b>: while held (and only when neither shoot trigger is held), runs the
 *       alternating-direction unjam sequence using {@link frc.robot.commands.commandGroups.LauncherUnJamCommandGroup}.</li>
 * </ul>
 *
 * <p>The launcher subsystem is optional to allow the robot to run even if the launcher is not
 * constructed/installed for a given configuration.
 */
public class SK26LauncherBinder implements CommandBinder {

    Optional<SK26Launcher> launcherSubsystem;

    Trigger ShootExitVel;
    Trigger ShootRPS;
    Trigger UnJam;

    /**
     * Creates a binder for the launcher subsystem.
     *
     * @param launcherSubsystem optional launcher subsystem instance
     */
    public SK26LauncherBinder(Optional<SK26Launcher> launcherSubsystem) {
        this.launcherSubsystem = launcherSubsystem;
        this.ShootExitVel = kShootExitVel.button;
        this.ShootRPS = kShootRPS.button;
        this.UnJam = kUnJam.button;
    }

    /**
     * Attaches commands to triggers if the launcher subsystem is present.
     *
     * <p>Note: The unjam command is gated so it will not run while either shoot trigger is active
     * to avoid conflicting motor commands.
     */
    @Override
    public void bindButtons() {

        if (launcherSubsystem.isPresent()) {

            SK26Launcher launcher = launcherSubsystem.get();

            ShootExitVel.whileTrue(new RunLauncherWithVelCommand(launcher, kTargetlaunchVelocity));
            ShootRPS.whileTrue(new RunLauncherWithRPSCommand(launcher, kTargetMotorRPS));
            UnJam.and(ShootRPS.negate()).and(ShootExitVel.negate())
                .whileTrue(new LauncherUnJamCommandGroup(launcher));
        }
    }

}
