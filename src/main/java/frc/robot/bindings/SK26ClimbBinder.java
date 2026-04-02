package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.subsystems.climb.SK26Climb;

import static frc.robot.Ports.OperatorPorts.kUpDpad;
import static frc.robot.Ports.OperatorPorts.kYbutton;
import static frc.robot.Ports.OperatorPorts.kBbutton;
import static frc.robot.Ports.OperatorPorts.kRightDpad;
import static frc.robot.Ports.OperatorPorts.kDownDpad;
import static frc.robot.Konstants.ClimbConstants.kTOne;
import static frc.robot.Konstants.ClimbConstants.kClimbReturn;

public class SK26ClimbBinder implements CommandBinder {

    Optional<SK26Climb> climbSubsystem;
    
    Trigger t1Button;
    Trigger upButton;
    Trigger downButton;
    Trigger returnButton;
    Trigger zeroButton;

    public SK26ClimbBinder(Optional<SK26Climb> climbSubsystem)
    {
        this.climbSubsystem = climbSubsystem;

        this.t1Button = kUpDpad.button;
        this.upButton = kYbutton.button;
        this.downButton = kBbutton.button;
        this.returnButton = kRightDpad.button;
        this.zeroButton = kDownDpad.button;
    }

    public void bindButtons()
    {
        if (climbSubsystem.isPresent())
        {
            SK26Climb climb = climbSubsystem.get();

            t1Button.onTrue(climb.climbToHeightCommand(kTOne).withName("L1ButtomClimb"));
            returnButton.onTrue(climb.climbToHeightCommand(kClimbReturn).withName("ClimbToClimbPosition"));
            zeroButton.onTrue(climb.climbToHeightCommand(0).withName("ClimbGoToZeroPosition"));
            upButton.whileTrue(climb.climbUpCommand().until(() -> climb.isForwardLimitReached()).withName("ClimbUpCommand"));
            downButton.whileTrue(climb.climbDownCommand().until(() -> climb.isReverseLimitReached()).withName("ClimbDownCommand"));
        }
    }
}

