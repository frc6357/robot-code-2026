package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.utils.SKTrigger;
import frc.robot.commands.ClimbButtonCommand;
import frc.robot.commands.ClimbDownCommand;
import frc.robot.commands.ClimbManualCommand;
import frc.robot.commands.ClimbUpCommand;
import frc.robot.subsystems.Climb;

import static frc.robot.Ports.OperatorPorts.climbGoButton;
import static frc.robot.Ports.OperatorPorts.climbUpButton;
import static frc.robot.Ports.OperatorPorts.climbDownButton;
import static frc.robot.Konstants.ClimbConstants.kTOne;
import static frc.robot.Konstants.ClimbConstants.kClimbReturn;

public class ClimbBinder implements CommandBinder {

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

            //t1Button.onTrue(new ClimbButtonCommand(kTOne, climb).withName("L1ButtomClimb"));
            t1Button.onTrue(Commands.sequence(new ClimbButtonCommand(kTOne, climb), new WaitCommand(0.5), new ClimbButtonCommand(kClimbReturn, climb)).withName("L1Command"));
            upButton.whileTrue(new ClimbUpCommand(climb).withName("ClimbUpCommand"));
            downButton.whileTrue(new ClimbDownCommand(climb).withName("ClimbDownCommand"));
        }
    }
}

