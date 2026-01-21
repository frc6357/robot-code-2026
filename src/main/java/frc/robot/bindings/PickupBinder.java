package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.*;
import static frc.robot.Ports.DriverPorts.*;

import frc.robot.subsystems.pickupOB.SK26PickupOB;
import frc.robot.commands.*;

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
    //Trigger stop;

    public PickupBinder(Optional<SK26PickupOB> pobSys) {
        subsystem = pobSys;
        extend = positionExtendButton.button;
        retract = positionRetractButton.button;
        eat = eaterEatButton.button;
        spit = eaterSpitButton.button;
        // stop = climbStopButton.button;
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
            //stop = whileTrue(new OBStopCommand(subsys));
        }
    }
}
