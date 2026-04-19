package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.StateHandler.MacroState.Status;
import frc.robot.commands.automatedDriving.ClimbApproachAndAlign;
import frc.robot.commands.automatedDriving.ClimbAlignment;
import frc.robot.subsystems.climb.SK26Climb;
import frc.robot.subsystems.drive.SKSwerve;

import static frc.robot.Ports.OperatorPorts.kUpDpad;
import static frc.robot.Ports.OperatorPorts.kYbutton;
import static frc.robot.Ports.OperatorPorts.kAbutton;
import static frc.robot.Ports.DriverPorts.kBackbutton;
import static frc.robot.Ports.OperatorPorts.kRightDpad;
import static frc.robot.Ports.OperatorPorts.kDownDpad;
import static frc.robot.Konstants.ClimbConstants.ClimbPosition.T_ONE;
import static frc.robot.Konstants.ClimbConstants.ClimbPosition.HOIST;
import static frc.robot.Konstants.ClimbConstants.ClimbPosition.STOW;

public class SK26ClimbBinder implements CommandBinder {

    Optional<SK26Climb> climbSubsystem;
    Optional<SKSwerve> swerveSubsystem;
    Optional<StateHandler> stateHandler;
    
    Trigger prepareForT1Button;
    Trigger upButton;
    Trigger downButton;
    Trigger hoistButton;
    Trigger stowButton;

    Trigger extendClimb;
    Trigger hoistClimb;

    public SK26ClimbBinder(Optional<SK26Climb> climbSubsystem, Optional<SKSwerve> swerveSubsystem, Optional<StateHandler> stateHandler)
    {
        this.climbSubsystem = climbSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        this.prepareForT1Button = kUpDpad.button;
        this.upButton = kYbutton.button;
        this.downButton = kAbutton.button;
        this.hoistButton = kDownDpad.button;
        this.stowButton = kRightDpad.button;

        if(stateHandler.isPresent()) {
            this.extendClimb = 
                StateHandler.whenStateHasStatus(MacroState.CLIMBING, Status.WAITING)
                .and(stateHandler.get().getIntakeAvoidingMajorFouls())
                .and(StateHandler.whenCurrentState(MacroState.CLIMBING).or(StateHandler.whenCurrentState(MacroState.CLIMB_AND_SCORE)));
            this.hoistClimb = StateHandler.whenStateHasStatus(MacroState.CLIMBING, Status.READY)
                .and(stateHandler.get().getIntakeAvoidingMajorFouls())
                 .and(StateHandler.whenCurrentState(MacroState.CLIMBING).or(StateHandler.whenCurrentState(MacroState.CLIMB_AND_SCORE)));
        }
    }

    public void bindButtons()
    {
        if (climbSubsystem.isPresent())
        {
            SK26Climb climb = climbSubsystem.get();

            // Set climbing state when climb button is pressed, then execute climb command
            prepareForT1Button.onTrue(
                climb.climbToHeightCommand(T_ONE)
                    .withName("ClimbPrepareForT1"));
            
            hoistButton.onTrue(
                climb.climbToHeightCommand(HOIST)
                    .withName("ClimbHoist"));
            
            // Stow climb and return to IDLE state
            stowButton.onTrue(
                climb.climbToHeightCommand(STOW)
                    .withName("ClimbStow"));
            
            upButton.whileTrue(climb.climbUpCommand().until(() -> climb.isForwardLimitReached()).withName("ClimbUpCommand"));
            downButton.whileTrue(climb.climbDownCommand().until(() -> climb.isReverseLimitReached()).withName("ClimbDownCommand"));

            hoistClimb.onTrue(climb.climbToHeightCommand(HOIST));

            // Automated climb sequence: when CLIMBING is requested and intake is stowed,
            // deploy arms to T_ONE and pathfind to tower approach pose in parallel
            if (extendClimb != null && swerveSubsystem.isPresent()) {
                SKSwerve drive = swerveSubsystem.get();
                extendClimb.whileTrue(
                    // Commands.deferredProxy(() ->
                        //Commands.defer(() ->
                            Commands.sequence(
                                // Commands.parallel(
                                //     climb.climbToHeightCommand(T_ONE).withName("ClimbTOne"),
                                //     ClimbApproachAndAlign.createPathfindCommand(drive).withName("TowerPathfind")
                                // ),
                                climb.climbToHeightCommand(T_ONE).withName("ClimbTOne"),
                                new ClimbAlignment(drive),
                                Commands.runOnce(() -> StateHandler.MacroState.CLIMBING.setStatus(Status.READY))
                            )
                            //Set.of(drive, climb)
                        //)
                    // , Set.of(drive, climb)
                );
                // );
                kBackbutton.button.whileTrue(ClimbApproachAndAlign.create(drive));
            } else if (extendClimb != null) {
                // No swerve available - just deploy arms
                extendClimb.onTrue(climb.climbToHeightCommand(T_ONE).withName("AutoClimbArmDeploy"));
            }

        }
    }
}

