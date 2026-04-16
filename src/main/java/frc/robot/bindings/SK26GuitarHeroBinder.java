package frc.robot.bindings;

import static frc.robot.Konstants.IntakeConstants.IntakePosition.GROUND;
import static frc.robot.Konstants.IntakeConstants.kIntakeFullVoltage;
import static frc.robot.Ports.GuitarHeroPorts.kGuitar;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.subsystems.climb.SK26Climb;
import frc.robot.subsystems.intake.SK26IntakePivot;
import frc.robot.subsystems.intake.SK26IntakeRollers;

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
    private final Optional<SK26IntakeRollers> intakeRollersSubsystem;
    private final Optional<SK26Climb> climbSubsystem;

    Trigger strumUp;
    Trigger strumDown;
    Trigger greenFret;
    Trigger redFret;
    Trigger yellowFret;
    Trigger backButton;

    public SK26GuitarHeroBinder(
            Optional<SK26IntakePivot> intakePivotSubsystem,
            Optional<SK26IntakeRollers> intakeRollersSubsystem,
            Optional<SK26Climb> climbSubsystem) {
        this.intakePivotSubsystem = intakePivotSubsystem;
        this.intakeRollersSubsystem = intakeRollersSubsystem;
        this.climbSubsystem = climbSubsystem;

        strumUp     = kGuitar.kUpDpad.button;
        strumDown   = kGuitar.kDownDpad.button;
        greenFret   = kGuitar.kAbutton.button;
        redFret     = kGuitar.kBbutton.button;
        yellowFret  = kGuitar.kYbutton.button;
        backButton  = kGuitar.kBackbutton.button;
    }

    @Override
    public void bindButtons() {
        // Climb controls
        if (climbSubsystem.isPresent()) {
            SK26Climb climb = climbSubsystem.get();

            greenFret.whileTrue(climb.climbUpCommand().withName("GuitarClimbUp"));
            redFret.whileTrue(climb.climbDownCommand().withName("GuitarClimbDown"));
            yellowFret.onTrue(climb.climbToHeightCommand(0).withName("GuitarClimbStow"));
        }

        // Intake pivot controls
        if (intakePivotSubsystem.isPresent()) {
            SK26IntakePivot pivot = intakePivotSubsystem.get();

            strumUp.onTrue(pivot.setIntakePivotTargetCommand(
                frc.robot.Konstants.IntakeConstants.IntakePosition.STOW).withName("GuitarIntakeStow"));
            strumDown.onTrue(pivot.setIntakePivotTargetCommand(GROUND).withName("GuitarIntakeGround"));
        }

        // Intake roller unjam (reverse voltage while held)
        if (intakeRollersSubsystem.isPresent()) {
            SK26IntakeRollers rollers = intakeRollersSubsystem.get();

            backButton.whileTrue(rollers.runAtVoltageCommand(-kIntakeFullVoltage).withName("GuitarIntakeUnjam"));
        }
    }
}
