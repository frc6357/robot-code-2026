package frc.robot.bindings;

import static frc.robot.Konstants.IntakeConstants.IntakePosition.COMPACTING;
import static frc.robot.Konstants.IntakeConstants.IntakePosition.GROUND;
import static frc.robot.Konstants.IntakeConstants.IntakePosition.STOW;
import static frc.robot.Ports.OperatorPorts;

import java.util.Optional;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
    Trigger intakeZeroPosition;
    Trigger trashCompact;

    public SK26IntakePivotBinder(Optional<SK26IntakePivot> pivotSubsystem)
    {
        this.pivotSubsystem = pivotSubsystem;

        // States that want the intake deployed
        intakingStates = StateHandler.whenCurrentState(MacroState.INTAKING)
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING))
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING));

        // States that want the intake stowed
        intakeZeroPosition = StateHandler.whenCurrentState(MacroState.CLIMBING)
            .or(StateHandler.whenCurrentState(MacroState.CLIMB_AND_SCORE));

        trashCompact = StateHandler.whenCurrentState(MacroState.SCORING).debounce(0.75, DebounceType.kRising);
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
        intakeZeroPosition.onTrue(pivot.setIntakePivotTargetCommand(STOW));

        trashCompact.whileTrue(new IntakeCompactCommand(pivot, GROUND.rotations, COMPACTING.rotations).withName("TrashCompactor"));

        /* Manual */
        // Pivoting
        OperatorPorts.kBackbutton.button.onTrue(pivot.setIntakePivotTargetCommand(STOW));
        OperatorPorts.kStartbutton.button.onTrue(pivot.setIntakePivotTargetCommand(GROUND));

        // Trash compactor
        // OperatorPorts.kRTrigger.button.whileTrue(new IntakeCompactCommand(pivot, GROUND.rotations, COMPACTING.rotations));
    }
}
