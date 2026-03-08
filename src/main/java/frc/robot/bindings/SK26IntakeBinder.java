package frc.robot.bindings;

import frc.lib.bindings.CommandBinder;
// Imports from robot
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.commands.*;
import frc.robot.subsystems.intake.SK26Intake;
import static frc.robot.Konstants.IntakeConstants.kIntakeFullSpeed;
import static frc.robot.Konstants.IntakeConstants.IntakePosition.kIntakeZeroPosition;

// Imports from Java/WPILib
import java.util.Optional;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SK26IntakeBinder implements CommandBinder 
{
    private final Optional<SK26Intake> intakeSubsystem;

    Trigger intakeRollersFullSpeed;
    Trigger intakeIdleSpeed;
    Trigger intakeZeroPosition;
    Trigger IsIdle;

    public SK26IntakeBinder(Optional<SK26Intake> intakeSubsystem) 
    {
        this.intakeSubsystem = intakeSubsystem;

        // For integration with other states
        intakeRollersFullSpeed = StateHandler.whenCurrentState(MacroState.INTAKING)
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING))
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING));
        intakeZeroPosition = StateHandler.whenCurrentState(MacroState.CLIMBING).or(StateHandler.whenCurrentState(MacroState.CLIMB_AND_SCORE));

        // For simple trigger bindings (if necessary)
        IsIdle = StateHandler.whenCurrentState(MacroState.IDLE);
    }

    public void bindButtons() 
    {
        if (intakeSubsystem.isEmpty())
        {
            return;
        }
        
        SK26Intake intake = intakeSubsystem.get();

        intakeRollersFullSpeed.whileTrue(new IntakeCommand(intake, kIntakeFullSpeed));
        // intakeIdleSpeed.whileTrue(new IntakeCommand(intake, kIntakeIdleSpeed)); There is no point for this. Just stop the intake
        intakeZeroPosition.whileTrue(new IntakePivotCommand(intake, kIntakeZeroPosition));
    }
}
