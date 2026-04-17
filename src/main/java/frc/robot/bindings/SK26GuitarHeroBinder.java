package frc.robot.bindings;

import static frc.robot.Konstants.ClimbConstants.ClimbPosition.T_ONE;
import static frc.robot.Konstants.ClimbConstants.ClimbPosition.HOIST;
import static frc.robot.Konstants.IntakeConstants.IntakePosition.GROUND;
import static frc.robot.Konstants.IntakeConstants.IntakePosition.STOW;
import static frc.robot.Ports.GuitarHeroPorts.kGuitar;

import java.util.Optional;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.subsystems.climb.SK26Climb;
import frc.robot.subsystems.indexer.SK26Indexer;
import frc.robot.subsystems.intake.SK26IntakePivot;

/**
 * Binds a Guitar Hero controller (mapped as an Xbox controller) to robot actions:
 * <ul>
 *   <li>Green fret  (A button)   — Climb up</li>
 *   <li>Red fret    (B button)   — Climb down</li>
 *   <li>Yellow fret (Y button)   — Climb stow</li>
 *   <li>Strum Up    (D-pad Up)   — Intake pivot stow</li>
 *   <li>Strum Down  (D-pad Down) — Intake pivot ground</li>
 *   <li>Back button              — Intake unjam</li>
 * </ul>
 */
public class SK26GuitarHeroBinder implements CommandBinder {

    private final Optional<SK26IntakePivot> intakePivotSubsystem;
    private final Optional<SK26Indexer> indexerSubsystem;
    private final Optional<SK26Climb> climbSubsystem;

    Trigger strumUp;
    Trigger strumDown;
    Trigger greenFret;
    Trigger redFret;
    Trigger yellowFret;
    Trigger backButton;

    Trigger IndexFeed;

    public SK26GuitarHeroBinder(
            Optional<SK26IntakePivot> intakePivotSubsystem,
            Optional<SK26Indexer> indexerSubsystem,
            Optional<SK26Climb> climbSubsystem) {
        this.intakePivotSubsystem = intakePivotSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.climbSubsystem = climbSubsystem;

        strumUp     = kGuitar.kUpDpad.button;
        strumDown   = kGuitar.kDownDpad.button;
        greenFret   = kGuitar.kAbutton.button;
        redFret     = kGuitar.kBbutton.button;
        yellowFret  = kGuitar.kYbutton.button;
        backButton  = kGuitar.kBackbutton.button;

        IndexFeed = StateHandler.whenCurrentStateReady(MacroState.SCORING)
            .or(StateHandler.whenCurrentStateReady(MacroState.SHUTTLING))
            .or(StateHandler.whenCurrentStateReady(MacroState.STEADY_STREAM_SCORING))
            .or(StateHandler.whenCurrentStateReady(MacroState.STEADY_STREAM_SHUTTLING));

    }

    @Override
    public void bindButtons() {
        // DEBUG: Test if triggers fire at all (check console for these prints)
        // Climb controls
        if (climbSubsystem.isPresent()) {
            SK26Climb climb = climbSubsystem.get();

            greenFret.onTrue(climb.climbToHeightCommand(T_ONE).withName("GuitarClimbUp"));
            redFret.onTrue(climb.climbToHeightCommand(HOIST).withName("GuitarClimbDown"));
            yellowFret.onTrue(climb.climbToHeightCommand(0).withName("GuitarClimbStow"));
        }

        // Intake pivot controls
        if (intakePivotSubsystem.isPresent()) {
            SK26IntakePivot pivot = intakePivotSubsystem.get();

            strumUp.onTrue(pivot.setIntakePivotTargetCommand(STOW).withName("GuitarIntakeStow"));
            strumDown.onTrue(pivot.setIntakePivotTargetCommand(GROUND).withName("GuitarIntakeGround"));
            
        }

        // Intake roller unjam (reverse voltage while held)
        if (indexerSubsystem.isPresent()) {
            SK26Indexer indexer = indexerSubsystem.get();

            backButton.onTrue(Commands.defer(() -> indexer.feedCommand(() -> 10.0), Set.of(indexer)));
            backButton.onFalse(Commands.defer(
                        () -> IndexFeed.getAsBoolean()
                            ? indexer.feedCommand(() -> 10.0)
                            : indexer.idleIndexerCommand(),
                        Set.of(indexer)));
        }
    }
}
