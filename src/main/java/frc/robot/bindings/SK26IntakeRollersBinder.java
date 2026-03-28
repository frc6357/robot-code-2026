package frc.robot.bindings;

import static frc.robot.Konstants.IntakeConstants.kIntakeFullVoltage;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.subsystems.intake.SK26IntakeRollers;
import frc.robot.subsystems.drive.SKSwerve;

public class SK26IntakeRollersBinder implements CommandBinder
{
    private final Optional<SK26IntakeRollers> rollersSubsystem;
    private final Optional<SKSwerve> swerveSubsystem;

    Trigger runIntakeRollers;
    Trigger trashCompact;
    Trigger spitting;

    public SK26IntakeRollersBinder(Optional<SK26IntakeRollers> rollersSubsystem, Optional<SKSwerve> swerveSubsystem)
    {
        this.rollersSubsystem = rollersSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        // States that want the rollers running
        runIntakeRollers = StateHandler.whenCurrentState(MacroState.INTAKING)
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING))
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING));

        trashCompact = StateHandler.whenCurrentState(MacroState.SCORING).or(StateHandler.whenCurrentState(MacroState.SHUTTLING));

        spitting = StateHandler.whenCurrentState(MacroState.SPITTING);
    }

    public void bindButtons()
    {
        if (rollersSubsystem.isEmpty())
        {
            return;
        }

        SK26IntakeRollers rollers = rollersSubsystem.get();

        /* State-based */
        if(swerveSubsystem.isEmpty()) {
            runIntakeRollers.whileTrue(rollers.runAtVoltageCommand(kIntakeFullVoltage).withName("IntakeRollersRun"));
        }
        else {
            runIntakeRollers.whileTrue(rollers.runBasedOnRobotSpeedCommand(() -> swerveSubsystem.get().getVelocity(false))
            .withName("IntakeRollersRunFromRobotVel"));
        }
        trashCompact.whileTrue(rollers.runAtVoltageCommand(kIntakeFullVoltage * 0.66).withName("IntakeRollersCompact"));
        spitting.whileTrue(rollers.runAtVoltageCommand(-kIntakeFullVoltage).withName("IntakeRollersSpit"));

        /* Manual */
        // OperatorPorts.kLTrigger.button.whileTrue(rollers.runAtVoltageCommand(kIntakeFullVoltage));
    }
}
