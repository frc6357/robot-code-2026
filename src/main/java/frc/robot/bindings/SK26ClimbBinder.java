package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.StateHandler.MacroState.Status;
import frc.robot.subsystems.climb.SK26Climb;
import frc.robot.subsystems.intake.SK26IntakePivot;

import static frc.robot.Ports.OperatorPorts.kUpDpad;
import static frc.robot.Ports.OperatorPorts.kYbutton;
import static frc.robot.Ports.OperatorPorts.kAbutton;
import static frc.robot.Ports.OperatorPorts.kRightDpad;
import static frc.robot.Ports.OperatorPorts.kDownDpad;
import static frc.robot.Konstants.ClimbConstants.ClimbPosition.T_ONE;
import static frc.robot.Konstants.ClimbConstants.ClimbPosition.HOIST;
import static frc.robot.Konstants.ClimbConstants.ClimbPosition.STOW;

public class SK26ClimbBinder implements CommandBinder {

    Optional<SK26Climb> climbSubsystem;
    
    Trigger prepareForT1Button;
    Trigger upButton;
    Trigger downButton;
    Trigger hoistButton;
    Trigger stowButton;

    Trigger extendClimb;

    public SK26ClimbBinder(Optional<SK26Climb> climbSubsystem, Optional<SK26IntakePivot> intakePivot)
    {
        this.climbSubsystem = climbSubsystem;

        this.prepareForT1Button = kUpDpad.button;
        this.upButton = kYbutton.button;
        this.downButton = kAbutton.button;
        this.hoistButton = kDownDpad.button;
        this.stowButton = kRightDpad.button;

        if(intakePivot.isPresent()) {
            this.extendClimb = StateHandler.whenStateHasStatus(MacroState.CLIMBING, Status.WAITING)
                .and(StateHandler.whenCurrentState(MacroState.CLIMBING).or(StateHandler.whenCurrentState(MacroState.CLIMB_AND_SCORE)));
        }
    }

    public void bindButtons()
    {
        if (climbSubsystem.isPresent())
        {
            SK26Climb climb = climbSubsystem.get();

            prepareForT1Button.onTrue(climb.climbToHeightCommand(T_ONE).withName("ClimbPrepareForT1"));
            hoistButton.onTrue(climb.climbToHeightCommand(HOIST).withName("ClimbHoist"));
            stowButton.onTrue(climb.climbToHeightCommand(STOW).withName("ClimbStow"));

            upButton.whileTrue(climb.climbUpCommand().until(() -> climb.isForwardLimitReached()).withName("ClimbUpCommand"));
            downButton.whileTrue(climb.climbDownCommand().until(() -> climb.isReverseLimitReached()).withName("ClimbDownCommand"));
        }
    }
}

