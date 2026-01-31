package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TurretJoystickCommand;
import frc.robot.commands.TurretTemporaryButtonCommand;
import frc.robot.subsystems.SK26Turret;
import frc.lib.utils.filters.LinearDeadbandFilter;

import static frc.robot.Konstants.TurretConstants.kManualTurretSpeed;
import static frc.robot.Konstants.TurretConstants.kTurretJoystickDeadband;
// import static frc.robot.Konstants.TurretConstants.kJoystickReversed;
import static frc.robot.Ports.OperatorPorts.kTurretAxis;
import static frc.robot.Ports.OperatorPorts.kLowAlgae;
import static frc.robot.Ports.OperatorPorts.kHighAlgae;

public class SK26TurretBinder implements CommandBinder
{
    private final Optional<SK26Turret> turretSubsystem;
    Trigger LowAlgae;
    Trigger HighAlgae;
    SlewRateLimiter slewLimiter;

    public SK26TurretBinder(Optional<SK26Turret> turretSubsystem)
    {
        this.turretSubsystem = turretSubsystem;
        this.LowAlgae = kLowAlgae.button;
        this.HighAlgae = kHighAlgae.button;
        slewLimiter = new SlewRateLimiter(kManualTurretSpeed * 1.75);
    }

    @Override
    public void bindButtons()
    {
        if (turretSubsystem.isEmpty())
        {
            return;
        }

        SK26Turret turret = turretSubsystem.get();

        // double joystickGain = kJoystickReversed ? -1.0 : 1.0;
        kTurretAxis.setFilter(new LinearDeadbandFilter(kTurretJoystickDeadband, 1.0));

        LowAlgae.whileTrue(new TurretTemporaryButtonCommand(90, turret));
        HighAlgae.whileTrue(new TurretTemporaryButtonCommand(0.0, turret));

        turret.setDefaultCommand(
            new TurretJoystickCommand(
                turret,
                () -> -slewLimiter.calculate(kTurretAxis.getFilteredAxis())
            )
        );
    }
}
