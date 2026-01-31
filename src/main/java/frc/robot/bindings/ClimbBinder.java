package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.utils.SKTrigger;
import frc.robot.commands.ClimbButtonCommand;
import frc.robot.commands.ClimbManualCommand;
import frc.robot.subsystems.Climb;

import static frc.robot.Ports.OperatorPorts.climbGoButton;
import static frc.robot.Ports.OperatorPorts.climbUpButton;
import static frc.robot.Ports.OperatorPorts.climbDownButton;
import static frc.robot.Konstants.ClimbConstants.kTOne;

public class ClimbBinder {

    Optional<Climb> climbSubsystem;
    
    Trigger t1Button;
    Trigger upButton;
    Trigger downButton;

    public ClimbBinder(Optional<Climb> climbSubsystem)
    {
        this.climbSubsystem = climbSubsystem;

        this.t1Button = climbGoButton.button;
        this.upButton = climbUpButton.button;
        this.downButton = climbDownButton.button;
    }

    public void bindButtons()
    {
        if (climbSubsystem.isPresent())
        {
            Climb climb = climbSubsystem.get();

            t1Button.whileTrue(new ClimbButtonCommand(kTOne, climb));
            //upButton.whileTrue(new ClimbManualCommand(270.0, climb));
        }
    }
}

