package frc.robot.bindings;

import static frc.robot.Konstants.IntakeConstants.kIntakeFullVoltage;

import static frc.robot.Ports.TesterPorts.kIntakeRollersButton;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.subsystems.intake.SK26IntakeRollers;

public class SK26IntakeRollersBinder implements CommandBinder
{
    private final Optional<SK26IntakeRollers> rollersSubsystem;

    Trigger intakeRollersFullSpeed;
    Trigger trashCompact;

    public SK26IntakeRollersBinder(Optional<SK26IntakeRollers> rollersSubsystem)
    {
        this.rollersSubsystem = rollersSubsystem;

        // States that want the rollers running
        intakeRollersFullSpeed = StateHandler.whenCurrentState(MacroState.INTAKING)
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING))
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING));

        trashCompact = StateHandler.whenCurrentState(MacroState.SCORING).or(StateHandler.whenCurrentState(MacroState.SHUTTLING));
    }

    public void bindButtons()
    {
        if (rollersSubsystem.isEmpty())
        {
            return;
        }

        SK26IntakeRollers rollers = rollersSubsystem.get();

        /* State-based */
        intakeRollersFullSpeed.whileTrue(rollers.runAtVoltageCommand(kIntakeFullVoltage).withName("IntakeRollersRun"));
        trashCompact.whileTrue(rollers.runAtVoltageCommand(kIntakeFullVoltage * 0.66).withName("IntakeRollersCompact"));

        kIntakeRollersButton.button.whileTrue(rollers.runAtVoltageCommand(kIntakeFullVoltage).withName("IntakeRollersRun"));
        /* Manual */
        // OperatorPorts.kLTrigger.button.whileTrue(rollers.runAtVoltageCommand(kIntakeFullVoltage));
    }
}
