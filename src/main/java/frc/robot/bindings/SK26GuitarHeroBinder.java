package frc.robot.bindings;

import static frc.robot.Konstants.IntakeConstants.IntakePosition.GROUND;
import static frc.robot.Konstants.IntakeConstants.IntakePosition.STOW;
import static frc.robot.Konstants.ClimbConstants.kTOne;
import static frc.robot.Konstants.ClimbConstants.kClimbReturn;
import static frc.robot.Ports.GuitarHeroPorts.kGuitar;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.subsystems.climb.SK26Climb;
import frc.robot.subsystems.intake.SK26IntakePivot;

/**
 * Binds a Guitar Hero controller to robot actions:
 * <ul>
 *   <li>Green fret  — Intake to ground position</li>
 *   <li>Red fret    — Intake stow</li>
 *   <li>Orange fret — Climb stow (height 0) and return to IDLE</li>
 *   <li>Strum Up    — Full climb up (to kTOne height)</li>
 *   <li>Strum Down  — Full climb down (to kClimbReturn height)</li>
 * </ul>
 */
public class SK26GuitarHeroBinder implements CommandBinder {

    private final Optional<SK26IntakePivot> intakePivotSubsystem;
    private final Optional<SK26Climb> climbSubsystem;
    private final Optional<StateHandler> stateHandler;

    Trigger greenButton;
    Trigger redButton;
    Trigger orangeButton;
    Trigger strumUp;
    Trigger strumDown;

    public SK26GuitarHeroBinder(
            Optional<SK26IntakePivot> intakePivotSubsystem,
            Optional<SK26Climb> climbSubsystem,
            Optional<StateHandler> stateHandler) {
        this.intakePivotSubsystem = intakePivotSubsystem;
        this.climbSubsystem = climbSubsystem;
        this.stateHandler = stateHandler;

        greenButton  = kGuitar.greenFret();
        redButton    = kGuitar.redFret();
        orangeButton = kGuitar.orangeFret();
        strumUp      = kGuitar.strumUp();
        strumDown    = kGuitar.strumDown();
    }

    @Override
    public void bindButtons() {
        // Intake pivot controls
        if (intakePivotSubsystem.isPresent()) {
            SK26IntakePivot pivot = intakePivotSubsystem.get();

            greenButton.onTrue(pivot.setIntakePivotTargetCommand(GROUND).withName("GuitarIntakeGround"));
            redButton.onTrue(pivot.setIntakePivotTargetCommand(STOW).withName("GuitarIntakeStow"));
        }

        // Climb controls
        if (climbSubsystem.isPresent()) {
            SK26Climb climb = climbSubsystem.get();

            orangeButton.onTrue(
                climb.climbToHeightCommand(0)
                    .andThen(Commands.runOnce(() -> stateHandler.ifPresent(sh -> sh.setCurrentState(MacroState.IDLE))))
                    .withName("GuitarClimbStow"));

            strumUp.onTrue(
                Commands.runOnce(() -> stateHandler.ifPresent(sh -> sh.setCurrentState(MacroState.CLIMBING)))
                    .andThen(climb.climbToHeightCommand(kTOne))
                    .withName("GuitarClimbUp"));

            strumDown.onTrue(
                Commands.runOnce(() -> stateHandler.ifPresent(sh -> sh.setCurrentState(MacroState.CLIMBING)))
                    .andThen(climb.climbToHeightCommand(kClimbReturn))
                    .withName("GuitarClimbDown"));
        }
    }
}
