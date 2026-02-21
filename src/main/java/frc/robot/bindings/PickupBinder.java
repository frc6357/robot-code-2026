package frc.robot.bindings;

import frc.robot.commands.*;
import frc.robot.subsystems.intake.SK26Intake;

import static frc.robot.Ports.OperatorPorts.kRightDpad;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PickupBinder implements CommandBinder {
    Optional<SK26Intake> subsystem;
    Trigger intake;

    public PickupBinder(Optional<SK26Intake> pobSys) {
        subsystem = pobSys;
        intake = kRightDpad.button;
    }

    public void bindButtons() 
    {
        if (subsystem.isPresent()) 
        {
            SK26Intake subsys = subsystem.get();

            intake.whileTrue(new IntakeCommand(subsys));
        }
    }
}
