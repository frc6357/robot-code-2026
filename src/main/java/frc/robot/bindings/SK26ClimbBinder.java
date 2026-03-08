package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimbButtonCommand;
import frc.robot.commands.ClimbDownCommand;

import frc.robot.commands.ClimbUpCommand;
import frc.robot.subsystems.climb.SK26Climb;

import static frc.robot.Ports.OperatorPorts.climbGoButton;
import static frc.robot.Ports.OperatorPorts.climbUpButton;
import static frc.robot.Ports.OperatorPorts.climbDownButton;
import static frc.robot.Ports.OperatorPorts.climbzeroButton;
import static frc.robot.Konstants.ClimbConstants.kTOne;
import static frc.robot.Konstants.ClimbConstants.kClimbReturn;

public class SK26ClimbBinder implements CommandBinder {

    Optional<SK26Climb> climbSubsystem;
    
    Trigger t1Button;
    Trigger upButton;
    Trigger downButton;
    Trigger returnButton;

    public SK26ClimbBinder(Optional<SK26Climb> climbSubsystem)
    {
        this.climbSubsystem = climbSubsystem;

        this.t1Button = climbGoButton.button;
        this.upButton = climbUpButton.button;
        this.downButton = climbDownButton.button;
        this.returnButton = climbzeroButton.button;
    }

    public void bindButtons()
    {
        if (climbSubsystem.isPresent())
        {
            SK26Climb climb = climbSubsystem.get();

            t1Button.onTrue(new ClimbButtonCommand(kTOne, climb).withName("L1ButtomClimb"));
            returnButton.onTrue(new ClimbButtonCommand(kClimbReturn, climb).withName("Returm to Home"));
            //t1Button.onTrue(Commands.sequence(new ClimbButtonCommand(kTOne, climb), new WaitCommand(0.5), new ClimbButtonCommand(kClimbReturn, climb)).withName("L1Command"));
            upButton.whileTrue(new ClimbUpCommand(climb).until(() -> climb.isForwardLimitReached()).withName("ClimbUpCommand"));
            downButton.whileTrue(new ClimbDownCommand(climb).until(() -> climb.isReverseLimitReached()).withName("ClimbDownCommand"));
        }
    }
}

