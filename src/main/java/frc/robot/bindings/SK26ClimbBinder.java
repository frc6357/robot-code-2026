package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
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
    Optional<StateHandler> stateHandler;
    
    Trigger t1Button;
    Trigger upButton;
    Trigger downButton;
    Trigger returnButton;
    Trigger zeroButton;

    public SK26ClimbBinder(Optional<SK26Climb> climbSubsystem, Optional<StateHandler> stateHandler)
    {
        this.climbSubsystem = climbSubsystem;
        this.stateHandler = stateHandler;

        this.t1Button = kUpDpad.button;
        this.upButton = kYbutton.button;
        this.downButton = kBbutton.button;
        this.returnButton = kDownDpad.button;
        this.zeroButton = kRightDpad.button;
    }

    public void bindButtons()
    {
        if (climbSubsystem.isPresent())
        {
            SK26Climb climb = climbSubsystem.get();

            // Set climbing state when climb button is pressed, then execute climb command
            t1Button.onTrue(
                Commands.runOnce(() -> stateHandler.ifPresent(sh -> sh.setCurrentState(MacroState.CLIMBING)))
                    .andThen(climb.climbToHeightCommand(kTOne))
                    .withName("L1ButtonClimb"));
            
            returnButton.onTrue(
                Commands.runOnce(() -> stateHandler.ifPresent(sh -> sh.setCurrentState(MacroState.CLIMBING)))
                    .andThen(climb.climbToHeightCommand(kClimbReturn))
                    .withName("ClimbToClimbPosition"));
            
            // Stow climb and return to IDLE state
            zeroButton.onTrue(
                climb.climbToHeightCommand(0)
                    .andThen(Commands.runOnce(() -> stateHandler.ifPresent(sh -> sh.setCurrentState(MacroState.IDLE))))
                    .withName("ClimbStowAndIdle"));
            
            upButton.whileTrue(climb.climbUpCommand().until(() -> climb.isForwardLimitReached()).withName("ClimbUpCommand"));
            downButton.whileTrue(climb.climbDownCommand().until(() -> climb.isReverseLimitReached()).withName("ClimbDownCommand"));
        }
    }
}

