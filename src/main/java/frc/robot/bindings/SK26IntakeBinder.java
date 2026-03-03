package frc.robot.bindings;

// Imports from robot
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.commands.*;
import frc.robot.subsystems.intake.SK26Intake;
import static frc.robot.Konstants.IntakeConstants.kIntakeFullSpeed;
import static frc.robot.Konstants.IntakeConstants.kIntakeIdleSpeed;
import static frc.robot.Konstants.IntakeConstants.IntakePosition.kIntakeZeroPosition;

// Imports from Java/WPILib
import java.util.Optional;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SK26IntakeBinder implements CommandBinder 
{
    private final Optional<SK26Intake> intakeSubsystem;

    Trigger intakeFullSpeed;
    Trigger intakeIdleSpeed;
    Trigger intakeZeroPosition;
    Trigger IsIdle;

    public SK26IntakeBinder(Optional<SK26Intake> intakeSubsystem) 
    {
        this.intakeSubsystem = intakeSubsystem;

        // For integration with other states
        intakeFullSpeed = StateHandler.whenCurrentState(MacroState.INTAKING)
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING))
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING));
        intakeIdleSpeed = StateHandler.whenCurrentState(MacroState.SCORING)
            .or(StateHandler.whenCurrentState(MacroState.SHUTTLING));
        intakeZeroPosition = StateHandler.whenCurrentState(MacroState.CLIMBING);

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

        intakeFullSpeed.whileTrue(new IntakeCommand(intake, kIntakeFullSpeed));
        intakeIdleSpeed.whileTrue(new IntakeCommand(intake, kIntakeIdleSpeed));
        intakeZeroPosition.whileTrue(new IntakePivotCommand(intake, kIntakeZeroPosition));
    }
}
