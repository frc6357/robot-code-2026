package frc.robot.bindings;

import java.util.Optional;
import frc.robot.commands.TurretJoystickCommand;
import frc.robot.subsystems.SK26Turret;
import static frc.robot.Ports.OperatorPorts.kTurretAxis;

public class SK26TurretBinder implements CommandBinder {

    private final Optional<SK26Turret> turretSubsystem;

    public SK26TurretBinder(Optional<SK26Turret> turretSubsystem) 
    {
        this.turretSubsystem = turretSubsystem;
    }

    @Override
    public void bindButtons() 
    {
        if (turretSubsystem.isPresent()) 
        {
            SK26Turret turret = turretSubsystem.get();

            turret.setDefaultCommand(
                new TurretJoystickCommand(
                    turret,
                    () -> kTurretAxis.getFilteredAxis()
                )
            );
        }
    }
}
