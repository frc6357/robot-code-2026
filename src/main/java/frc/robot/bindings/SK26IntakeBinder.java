package frc.robot.bindings;

import frc.lib.bindings.CommandBinder;
// Imports from robot
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.commands.*;
import frc.robot.subsystems.intake.SK26Intake;

import static frc.robot.Konstants.IntakeConstants.kIntakeFullVoltage;
import static frc.robot.Konstants.IntakeConstants.kIntakeIdleVoltage;
import static frc.robot.Konstants.IntakeConstants.IntakePosition.kIntakeZeroPosition;
import static frc.robot.Konstants.IntakeConstants.IntakePosition.kIntakeGroundPosition;
import static frc.robot.Ports.OperatorPorts.kBackbutton;
import static frc.robot.Ports.OperatorPorts.kLTrigger;
import static frc.robot.Ports.OperatorPorts.kStartbutton;

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

        // intakeRollersFullSpeed.whileTrue(new IntakeCommand(intake, kIntakeFullVoltage));
        kLTrigger.button.whileTrue(new IntakeCommand(intake, kIntakeFullVoltage));
        // intakeIdleSpeed.whileTrue(new IntakeCommand(intake, kIntakeIdleVoltage));
        kBackbutton.button.onTrue(new IntakePivotCommand(intake, kIntakeZeroPosition));
        kStartbutton.button.onTrue(new IntakePivotCommand(intake, kIntakeGroundPosition));
    }
}
