package frc.robot.bindings;

import java.util.Optional;

import frc.lib.utils.filters.Filter;
import frc.robot.commands.TurretJoystickCommand;
import frc.robot.subsystems.SK26Turret;
import frc.robot.utils.filters.DeadbandFilter;

import static frc.robot.Konstants.TurretConstants.kTurretDeadband;
import static frc.robot.Konstants.TurretConstants.kJoystickReversed;
import static frc.robot.Ports.OperatorPorts.kTurretAxis;

public class SK26TurretBinder implements CommandBinder
{
    private final Optional<SK26Turret> turretSubsystem;

    public SK26TurretBinder(Optional<SK26Turret> turretSubsystem)
    {
        this.turretSubsystem = turretSubsystem;
    }

    @Override
    public void bindButtons()
    {
        if (turretSubsystem.isEmpty())
        {
            return;
        }

        SK26Turret turret = turretSubsystem.get();

        double joystickGain = kJoystickReversed ? -1.0 : 1.0;
        kTurretAxis.setFilter((Filter) new DeadbandFilter(kTurretDeadband, joystickGain));

        turret.setDefaultCommand(
            new TurretJoystickCommand(
                turret,
                () -> kTurretAxis.getFilteredAxis()
            )
        );
    }
}
