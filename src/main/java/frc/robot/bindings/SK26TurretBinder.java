package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TurretJoystickCommand;
import frc.robot.commands.TurretButtonCommand;
import frc.robot.commands.TurretTrackPointCommand;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.turret.SK26Turret;
import frc.lib.utils.Field;
import frc.lib.utils.filters.LinearDeadbandFilter;

import static frc.robot.Konstants.TargetPointConstants.TargetPoint.kBlueHub;
import static frc.robot.Konstants.TargetPointConstants.TargetPoint.kRedHub;
import static frc.robot.Konstants.TargetPointConstants.TargetPoint.kOperatorControlled;
import static frc.robot.Konstants.TurretConstants.kManualTurretSpeed;
import static frc.robot.Konstants.TurretConstants.kTurretJoystickDeadband;
import static frc.robot.Konstants.TurretConstants.TurretPosition.kTurretZeroPosition;
import static frc.robot.Konstants.TurretConstants.TurretPosition.kTurretLeftPosition;
import static frc.robot.Ports.OperatorPorts.kRightStickX;
import static frc.robot.Ports.OperatorPorts.kAbutton;
import static frc.robot.Ports.OperatorPorts.kBbutton;
import static frc.robot.Ports.OperatorPorts.kYbutton;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;

public class SK26TurretBinder implements CommandBinder
{
    private final Optional<SK26Turret> turretSubsystem;
    private final Optional<SKSwerve> swerveSubsystem;

    Trigger PointAtShuttlePoint;
    Trigger PointAtHub;
    Trigger IsIdle;

    SlewRateLimiter slewLimiter;

    public SK26TurretBinder(Optional<SK26Turret> turretSubsystem, Optional<SKSwerve> swerveSubsystem)
    {
        this.turretSubsystem = turretSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        slewLimiter = new SlewRateLimiter(kManualTurretSpeed * 1.75);

        PointAtHub = StateHandler.whenCurrentState(MacroState.SCORING)
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING));

        PointAtShuttlePoint = StateHandler.whenCurrentState(MacroState.SHUTTLING)
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING));

        IsIdle = StateHandler.whenCurrentState(MacroState.IDLE);
    }

    @Override
    public void bindButtons()
    {
        if (turretSubsystem.isEmpty() || swerveSubsystem.isEmpty())
        {
            return;
        }

        SK26Turret turret = turretSubsystem.get();
        SKSwerve swerve = swerveSubsystem.get();

        kRightStickX.setFilter(new LinearDeadbandFilter(kTurretJoystickDeadband, 1.0));

        kBbutton.button.and(IsIdle).whileTrue(new TurretButtonCommand(kTurretLeftPosition, turret));
        kYbutton.button.and(IsIdle).whileTrue(new TurretButtonCommand(kTurretZeroPosition, turret));
        kAbutton.button.and(IsIdle).toggleOnTrue(new TurretTrackPointCommand(
            turret, 
            swerve,
            kOperatorControlled.point
            // Field.isBlue() ? kBlueHub.point : kRedHub.point
        ).withName("TurretManualTrackHubCommand"));

        PointAtHub.whileTrue(new TurretTrackPointCommand(turret, swerve, Field.isBlue() ? kBlueHub.point : kRedHub.point)
            .withName("TurretTrackHubCommand"));
        PointAtShuttlePoint.whileTrue(new TurretTrackPointCommand(turret, swerve, kOperatorControlled.point)
            .withName("TurretTrackShuttleCommand"));


        // Default command: Use the right joystick to manually move the turret when idle
        turret.setDefaultCommand(
            new TurretJoystickCommand(
                turret, 
                () -> slewLimiter.calculate(-kRightStickX.getFilteredAxis()))
            .unless(IsIdle.negate()).withName("TurretManualJoystick")
        );
    }
}
