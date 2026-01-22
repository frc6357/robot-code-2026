package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TurretJoystickCommand;
import frc.robot.commands.TurretJoystickCommand2;
import frc.robot.subsystems.SK26Turret;
import static frc.robot.Ports.OperatorPorts.kFloorAlgae;
import static frc.robot.Ports.OperatorPorts.kLowAlgae;
import static frc.robot.Ports.OperatorPorts.kTurretAxis;;

public class SK26TurretBinder implements CommandBinder {

    private final Optional<SK26Turret> turretSubsystem;
    Trigger move;
    Trigger moveOpposite;

    public SK26TurretBinder(Optional<SK26Turret> turretSubsystem) 
    {
        this.turretSubsystem = turretSubsystem;
        move = kFloorAlgae.button;
        moveOpposite = kLowAlgae.button;

    }

    @Override
    public void bindButtons() 
    {
        if (turretSubsystem.isPresent()) 
        {
            SK26Turret turret = turretSubsystem.get();

            move.whileTrue(new TurretJoystickCommand(turret));
            moveOpposite.whileTrue(new TurretJoystickCommand2(turret));
        }
    }
}
