package frc.robot.bindings;

import static frc.robot.Konstants.TargetPointConstants.TargetPoint.kOperatorControlled;
import static frc.robot.Konstants.TurretConstants.kManualTurretSpeed;
import static frc.robot.Konstants.TurretConstants.kTurretJoystickDeadband;
import static frc.robot.Ports.OperatorPorts.kRightStickX;

import java.util.Optional;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.lib.utils.Field;
import frc.lib.utils.FieldConstants;
import frc.lib.utils.FieldConstants.LinesVertical;
import frc.lib.utils.filters.LinearDeadbandFilter;
import frc.robot.Konstants.SwerveConstants;
import frc.robot.RobotContainer;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.commands.turret.TurretJoystickCommand;
import frc.robot.commands.turret.TurretTrackPointCommand;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.turret.SK26Turret;

public class SK26TurretBinder implements CommandBinder
{
    private final Optional<SK26Turret> turretSubsystem;
    private final Optional<SKSwerve> swerveSubsystem;

    Trigger PointAtShuttlePoint;
    Trigger PointAtHub;
    Trigger IsIdle;

    Trigger inAllianceZone;
    Trigger outOfAllianceZone;

    SlewRateLimiter slewLimiter;

    public SK26TurretBinder(Optional<SK26Turret> turretSubsystem, Optional<SKSwerve> swerveSubsystem)
    {
        this.turretSubsystem = turretSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        slewLimiter = new SlewRateLimiter(kManualTurretSpeed * 1.75);

        PointAtHub = (StateHandler.whenCurrentState(MacroState.SCORING)
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING))).and(() -> RobotContainer.m_shootingCoordinatorInstance == null);

        PointAtShuttlePoint = StateHandler.whenCurrentState(MacroState.SHUTTLING)
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING));

        IsIdle = StateHandler.whenCurrentState(MacroState.IDLE);

        if(swerveSubsystem.isEmpty()) {
            return;
        }
        inAllianceZone = new Trigger(() -> {
            double robotX = swerveSubsystem.get().getRobotPose().getX();
            if (Field.isBlue()) {
                return robotX < LinesVertical.allianceZone + Units.inchesToMeters(SwerveConstants.kChassisLength / 2.0);
            } else {
                return robotX > LinesVertical.redAllianceZone - Units.inchesToMeters(SwerveConstants.kChassisLength / 2.0);
            }
        });
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

        // Using operator controller buttons:

        kRightStickX.setFilter(new LinearDeadbandFilter(kTurretJoystickDeadband, 1.0));

        // kBbutton.button.and(IsIdle).whileTrue(new TurretButtonCommand(kTurretLeftPosition, turret));
        // kYbutton.button.and(IsIdle).whileTrue(new TurretButtonCommand(kTurretZeroPosition, turret));
        // kAbutton.button.and(IsIdle).toggleOnTrue(new TurretTrackPointCommand(
        //     turret, 
        //     swerve,
        //     kOperatorControlled.point
        //     // Field.isBlue() ? kBlueHub.point : kRedHub.point
        // ).withName("TurretManualTrackHubCommand"));

        inAllianceZone.negate().and(() -> DriverStation.isEnabled()).whileTrue(
            new TurretTrackPointCommand(turret, swerve, kOperatorControlled.point)
            .withName("TurretTrackOperatorCommand"));
        
        inAllianceZone.and(() -> DriverStation.isEnabled()).whileTrue(Commands.deferredProxy(() -> 
            new TurretTrackPointCommand(turret, swerve, Field.isBlue() ? FieldConstants.Hub.topCenterPoint.toTranslation2d() :
                                                                        FieldConstants.Hub.redTopCenterPoint.toTranslation2d())
        ).withName("TurretTrackHub" + DriverStation.getAlliance().get()));


        // Default command: Use the right joystick to manually move the turret when idle
        turret.setDefaultCommand(
            new TurretJoystickCommand(
                turret, 
                () -> slewLimiter.calculate(-kRightStickX.getFilteredAxis()))
            .unless(IsIdle.negate()).withName("TurretManualJoystick")
        );
    }
}
