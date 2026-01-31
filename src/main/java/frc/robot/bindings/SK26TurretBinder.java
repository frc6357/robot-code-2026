package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TurretTemporaryButtonCommand;
import frc.robot.commands.TurretTrackPointCommand;
import frc.robot.subsystems.SK26Turret;
import frc.robot.subsystems.drive.SKSwerve;
import frc.lib.utils.filters.LinearDeadbandFilter;

import static frc.robot.Konstants.TargetPointConstants.TargetPoint.kOperatorControlled;
import static frc.robot.Konstants.TargetPointConstants.targetPoints;
import static frc.robot.Konstants.TurretConstants.kManualTurretSpeed;
import static frc.robot.Konstants.TurretConstants.kTurretJoystickDeadband;
import static frc.robot.Ports.OperatorPorts.kTurretAxis;
import static frc.robot.Ports.OperatorPorts.kLowAlgae;
import static frc.robot.Ports.OperatorPorts.kHighAlgae;

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
        this.LowAlgae = kLowAlgae.button;
        this.HighAlgae = kHighAlgae.button;
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

        kTurretAxis.setFilter(new LinearDeadbandFilter(kTurretJoystickDeadband, 1.0));

        LowAlgae.whileTrue(new TurretTemporaryButtonCommand(90, turret));
        HighAlgae.whileTrue(new TurretTemporaryButtonCommand(0.0, turret));

        // Default command: continuously track the operator-controlled target point
        turret.setDefaultCommand(
            new TurretTrackPointCommand(turret, swerve,
                targetPoints[kOperatorControlled.ordinal()]
            )
        );
    }
}
