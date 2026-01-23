package frc.robot.bindings;

import java.util.Optional;

import frc.robot.commands.TurretJoystickCommand;
import frc.robot.subsystems.SK26Turret;
import frc.lib.utils.filters.LinearDeadbandFilter;

import static frc.robot.Konstants.TurretConstants.kTurretDeadband;
// import static frc.robot.Konstants.TurretConstants.kJoystickReversed;
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

        // double joystickGain = kJoystickReversed ? -1.0 : 1.0;
        kTurretAxis.setFilter(new LinearDeadbandFilter(kTurretDeadband, 1.0));

        turret.setDefaultCommand(
            new TurretJoystickCommand(
                turret,
                () -> -kTurretAxis.getFilteredAxis() //TODO: Determine if axis should be inverted (it probably should)
            )
        );
    }
}
