package frc.robot.bindings;

import static frc.robot.Konstants.IntakeConstants.IntakePosition.COMPACTING;
import static frc.robot.Konstants.IntakeConstants.IntakePosition.FULL_STOW;
import static frc.robot.Konstants.IntakeConstants.IntakePosition.GROUND;
import static frc.robot.Konstants.IntakeConstants.IntakePosition.STOW;
import static frc.robot.Ports.OperatorPorts;

import java.util.Optional;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.commands.IntakeCompactCommand;
import frc.robot.subsystems.intake.SK26IntakePivot;

public class SK26IntakePivotBinder implements CommandBinder
{
    private final Optional<SK26IntakePivot> pivotSubsystem;

    Trigger intakingStates;
    Trigger intakeAvoidMajorFoulsPosition;
    Trigger trashCompact;

    public SK26IntakePivotBinder(Optional<SK26IntakePivot> pivotSubsystem, Optional<StateHandler> stateHandlerContainer)
    {
        this.pivotSubsystem = pivotSubsystem;

        // States that want the intake deployed
        intakingStates = StateHandler.whenCurrentState(MacroState.INTAKING)
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING))
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING));

        // States that want the intake stowed
        intakeAvoidMajorFoulsPosition = StateHandler.whenCurrentState(MacroState.CLIMBING)
            .or(StateHandler.whenCurrentState(MacroState.CLIMB_AND_SCORE));

        if(stateHandlerContainer.isPresent()) {
            intakingStates = intakingStates.and(stateHandlerContainer.get().getClimbStowed());
        }

        trashCompact = StateHandler.whenCurrentState(MacroState.SCORING).or(StateHandler.whenCurrentState(MacroState.SHUTTLING)).debounce(0.75, DebounceType.kRising);
    }

    public void bindButtons()
    {
        if (pivotSubsystem.isEmpty())
        {
            return;
        }

        SK26IntakePivot pivot = pivotSubsystem.get();

        /* State-based */
        // Auto-deploy to GROUND when entering an intaking state and pivot is currently at ZERO
        intakingStates.and(() -> (pivot.getPositionerTargetEnum() == STOW))
            .onTrue(pivot.setIntakePivotTargetCommand(GROUND).withName("IntakeDeploy"));

        // Stow when climbing
        intakeAvoidMajorFoulsPosition.onTrue(
            Commands.sequence(
                pivot.setIntakePivotTargetCommand(FULL_STOW),
                Commands.waitUntil(() -> pivot.getCurrentPosition() >= 0.0),
                pivot.setIntakePivotTargetCommand(STOW)
            ).handleInterrupt(() -> pivot.setPositionerPosition(STOW))
            .withName("IntakeStowForClimb"));

        trashCompact.whileTrue(new IntakeCompactCommand(pivot, GROUND.rotations, COMPACTING.rotations).withName("TrashCompactor"));


        /* Manual */
        // Pivoting
        OperatorPorts.kBackbutton.button.onTrue(pivot.setIntakePivotTargetCommand(STOW));
        OperatorPorts.kStartbutton.button.onTrue(pivot.setIntakePivotTargetCommand(GROUND));

        // Trash compactor
        // OperatorPorts.kRTrigger.button.whileTrue(new IntakeCompactCommand(pivot, GROUND.rotations, COMPACTING.rotations));
    }
}
