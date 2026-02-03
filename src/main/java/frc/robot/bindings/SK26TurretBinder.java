package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TurretTemporaryButtonCommand;
import frc.robot.commands.TurretTrackPointCommand;
import frc.robot.subsystems.SK26Turret;
import frc.robot.subsystems.drive.SKSwerve;
import frc.lib.utils.Field;
import frc.lib.utils.filters.LinearDeadbandFilter;

import static frc.robot.Konstants.TargetPointConstants.TargetPoint.kBlueHub;
import static frc.robot.Konstants.TargetPointConstants.TargetPoint.kRedHub;
import static frc.robot.Konstants.TargetPointConstants.TargetPoint.kOperatorControlled;
import static frc.robot.Konstants.TargetPointConstants.targetPoints;
import static frc.robot.Konstants.TurretConstants.kManualTurretSpeed;
import static frc.robot.Konstants.TurretConstants.kTurretJoystickDeadband;
import static frc.robot.Ports.OperatorPorts.kRightStickX;
import static frc.robot.Ports.OperatorPorts.kAbutton;
import static frc.robot.Ports.OperatorPorts.kBbutton;
import static frc.robot.Ports.OperatorPorts.kYbutton;

public class SK26TurretBinder implements CommandBinder
{
    private final Optional<SK26Turret> turretSubsystem;
    private final Optional<SKSwerve> swerveSubsystem;
    Trigger LowAlgae;
    Trigger HighAlgae;
    SlewRateLimiter slewLimiter;

    public SK26TurretBinder(Optional<SK26Turret> turretSubsystem, Optional<SKSwerve> swerveSubsystem)
    {
        this.turretSubsystem = turretSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        slewLimiter = new SlewRateLimiter(kManualTurretSpeed * 1.75);
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

        kBbutton.button.whileTrue(new TurretTemporaryButtonCommand(90, turret));
        kYbutton.button.whileTrue(new TurretTemporaryButtonCommand(0.0, turret));
        kAbutton.button.whileTrue(new TurretTrackPointCommand(
            turret, 
            swerve, 
            Field.isBlue() ? targetPoints[kBlueHub.ordinal()] : targetPoints[kRedHub.ordinal()]
        ));


        // Default command: continuously track the operator-controlled target point
        turret.setDefaultCommand(
            new TurretTrackPointCommand(turret, swerve,
                targetPoints[kOperatorControlled.ordinal()]
            )
        );
    }
}
