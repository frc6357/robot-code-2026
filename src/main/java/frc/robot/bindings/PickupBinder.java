package frc.robot.bindings;

import frc.robot.subsystems.pickupOB.SK26PickupOB;
import frc.robot.commands.*;

import static frc.robot.Ports.OperatorPorts.kDownDpad;
import static frc.robot.Ports.OperatorPorts.kLeftDpad;
import static frc.robot.Ports.OperatorPorts.kRightDpad;
import static frc.robot.Ports.OperatorPorts.kUpDpad;

import java.util.Optional;

//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.RepeatCommand;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PickupBinder implements CommandBinder {
    Optional<SK26PickupOB> subsystem;
    Trigger extend;
    Trigger retract;
    Trigger eat;
    Trigger spit;

    public PickupBinder(Optional<SK26PickupOB> pobSys) {
        subsystem = pobSys;
        extend = kRightDpad.button;
        retract = kLeftDpad.button;
        eat = kUpDpad.button;
        spit = kDownDpad.button;
    }

    public void bindButtons() 
    {
        if (subsystem.isPresent()) 
        {
            SK26PickupOB subsys = subsystem.get();

            extend.whileTrue(new OBPickupPositionCommand(subsys)); 
            retract.whileTrue(new OBPickupRetractCommand(subsys));
            eat.whileTrue(new OBPickupEatCommand(subsys));
            spit.whileTrue(new OBPickupSpitCommand(subsys));
            //stop = onTrue(new OBStopCommand(subsys));
        }
    }
}
