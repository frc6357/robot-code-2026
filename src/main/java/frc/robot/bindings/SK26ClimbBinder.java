package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.subsystems.climb.SK26Climb;

import static frc.robot.Ports.OperatorPorts.climbTOneButton;
import static frc.robot.Ports.OperatorPorts.climbUpButton;
import static frc.robot.Ports.OperatorPorts.climbDownButton;
import static frc.robot.Ports.OperatorPorts.climbzeroButton;
import static frc.robot.Konstants.ClimbConstants.kTOne;
import static frc.robot.Konstants.ClimbConstants.kClimbReturn;

import static frc.robot.Ports.TesterPorts.kClimbTOneButton;
import static frc.robot.Ports.TesterPorts.kClimbReturnButton;

public class SK26ClimbBinder implements CommandBinder {

    Optional<SK26Climb> climbSubsystem;
    
    Trigger t1Button;
    Trigger upButton;
    Trigger downButton;
    Trigger returnButton;

    Trigger testT1Button;
    Trigger testReturnButton;

    public SK26ClimbBinder(Optional<SK26Climb> climbSubsystem)
    {
        this.climbSubsystem = climbSubsystem;

        this.t1Button = climbTOneButton.button;
        this.upButton = climbUpButton.button;
        this.downButton = climbDownButton.button;
        this.returnButton = climbzeroButton.button;

        this.testT1Button = kClimbTOneButton.button;
        this.testReturnButton = kClimbReturnButton.button;
    }

    public void bindButtons()
    {
        if (climbSubsystem.isPresent())
        {
            SK26Climb climb = climbSubsystem.get();

            t1Button.onTrue(climb.climbToHeightCommand(kTOne).withName("L1ButtomClimb"));
            returnButton.onTrue(climb.climbToHeightCommand(kClimbReturn).withName("Returm to Home"));
            //t1Button.onTrue(Commands.sequence(new ClimbButtonCommand(kTOne, climb), new WaitCommand(0.5), new ClimbButtonCommand(kClimbReturn, climb)).withName("L1Command"));
            upButton.whileTrue(climb.climbUpCommand().until(() -> climb.isForwardLimitReached()).withName("ClimbUpCommand"));
            downButton.whileTrue(climb.climbDownCommand().until(() -> climb.isReverseLimitReached()).withName("ClimbDownCommand"));

            testT1Button.onTrue(climb.climbToHeightCommand(kTOne).withName("ClimbButtonL1Test"));
            testReturnButton.onTrue(climb.climbToHeightCommand(kClimbReturn).withName("ClimbButtonReturnTest"));
        }
    }
}

