package frc.robot.bindings;

import static frc.robot.Konstants.IntakeConstants.IntakePosition.GROUND;
import static frc.robot.Konstants.IntakeConstants.IntakePosition.STOW;
import static frc.robot.Konstants.IntakeConstants.kIntakeFullVoltage;
import static frc.robot.Ports.GuitarHeroPorts.kGuitar;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.subsystems.intake.SK26IntakePivot;
import frc.robot.subsystems.intake.SK26IntakeRollers;

/**
 * Binds a Guitar Hero controller (mapped as an Xbox controller) to robot actions:
 * <ul>
 *   <li>Strum Up   (D-pad Up)   — Stow intake</li>
 *   <li>Strum Down (D-pad Down) — Ground position intake</li>
 *   <li>Green fret (A button)   — Unjam intake rollers (reversed voltage while held)</li>
 * </ul>
 */
public class SK26GuitarHeroBinder implements CommandBinder {

    private final Optional<SK26IntakePivot> intakePivotSubsystem;
    private final Optional<SK26IntakeRollers> intakeRollersSubsystem;

    Trigger strumUp;
    Trigger strumDown;
    Trigger greenFret;

    public SK26GuitarHeroBinder(
            Optional<SK26IntakePivot> intakePivotSubsystem,
            Optional<SK26IntakeRollers> intakeRollersSubsystem) {
        this.intakePivotSubsystem = intakePivotSubsystem;
        this.intakeRollersSubsystem = intakeRollersSubsystem;

        strumUp    = kGuitar.kUpDpad.button;
        strumDown  = kGuitar.kDownDpad.button;
        greenFret  = kGuitar.kAbutton.button;
    }

    @Override
    public void bindButtons() {
        // Intake pivot controls
        if (intakePivotSubsystem.isPresent()) {
            SK26IntakePivot pivot = intakePivotSubsystem.get();

            strumUp.onTrue(pivot.setIntakePivotTargetCommand(STOW).withName("GuitarIntakeStow"));
            strumDown.onTrue(pivot.setIntakePivotTargetCommand(GROUND).withName("GuitarIntakeGround"));
        }

        // Intake roller unjam (reverse voltage while held)
        if (intakeRollersSubsystem.isPresent()) {
            SK26IntakeRollers rollers = intakeRollersSubsystem.get();

            greenFret.whileTrue(rollers.runAtVoltageCommand(-kIntakeFullVoltage).withName("GuitarIntakeUnjam"));
        }
    }
}
