package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TurretJoystickCommand;
import frc.robot.commands.TurretTemporaryButtonCommand;
import frc.robot.subsystems.SK26Turret;
import frc.lib.utils.filters.LinearDeadbandFilter;

import static frc.robot.Konstants.TurretConstants.kTurretDeadband;
// import static frc.robot.Konstants.TurretConstants.kJoystickReversed;
import static frc.robot.Ports.OperatorPorts.kTurretAxis;
import static frc.robot.Ports.OperatorPorts.kLowAlgae;
import static frc.robot.Ports.OperatorPorts.kHighAlgae;

public class SK26TurretBinder implements CommandBinder
{
    private final Optional<SK26Turret> turretSubsystem;
    Trigger LowAlgae;
    Trigger HighAlgae;

    public SK26TurretBinder(Optional<SK26Turret> turretSubsystem)
    {
        this.turretSubsystem = turretSubsystem;
        this.LowAlgae = kLowAlgae.button;
        this.HighAlgae = kHighAlgae.button;
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

        LowAlgae.onTrue(new TurretTemporaryButtonCommand(90, turret));
        HighAlgae.onTrue(new TurretTemporaryButtonCommand(0.0, turret));

        turret.setDefaultCommand(
            new TurretJoystickCommand(
                turret,
                () -> -kTurretAxis.getFilteredAxis() //TODO: Determine if axis should be inverted (it probably should)
            )
        );
    }
}
