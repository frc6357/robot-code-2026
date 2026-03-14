// package frc.robot.bindings;

// import static frc.robot.Konstants.IntakeConstants.kIntakeFullVoltage;
// import static frc.robot.Konstants.IntakeConstants.IntakePosition.COMPACTING;
// import static frc.robot.Konstants.IntakeConstants.IntakePosition.GROUND;
// import static frc.robot.Konstants.IntakeConstants.IntakePosition.ZERO;
// import static frc.robot.Ports.OperatorPorts;

// // Imports from Java/WPILib
// import java.util.Optional;

// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.lib.bindings.CommandBinder;
// import frc.robot.StateHandler;
// import frc.robot.StateHandler.MacroState;
// import frc.robot.commands.IntakeCompactCommand;
// import frc.robot.subsystems.intake.SK26Intake;

// public class SK26IntakeBinder implements CommandBinder 
// {
//     private final Optional<SK26Intake> intakeSubsystem;

//     Trigger intakeRollersFullSpeed;
//     Trigger intakeIdleSpeed;
//     Trigger intakeZeroPosition;
//     Trigger IsIdle;

//     public SK26IntakeBinder(Optional<SK26Intake> intakeSubsystem) 
//     {
//         this.intakeSubsystem = intakeSubsystem;

//         // For integration with other states
//         intakeRollersFullSpeed = StateHandler.whenCurrentState(MacroState.INTAKING)
//             .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING))
//             .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING));
//         intakeZeroPosition = StateHandler.whenCurrentState(MacroState.CLIMBING).or(StateHandler.whenCurrentState(MacroState.CLIMB_AND_SCORE));

//         // For simple trigger bindings (if necessary)
//         IsIdle = StateHandler.whenCurrentState(MacroState.IDLE);
//     }

//     public void bindButtons() 
//     {
//         if (intakeSubsystem.isEmpty())
//         {
//             return;
//         }
        
//         SK26Intake intake = intakeSubsystem.get();

//         /* State-based */
//         // Rollers
//         intakeRollersFullSpeed.whileTrue(intake.runAtVoltageCommand(kIntakeFullVoltage));

//         intakeRollersFullSpeed.and(() -> (intake.getPositionerTargetEnum() == ZERO)).onTrue(intake.setIntakePivotTargetCommand(GROUND));
//         // Pivoting
//         intakeZeroPosition.onTrue(intake.setIntakePivotTargetCommand(ZERO));



//         /* Manual */
//         // Rollers
//         OperatorPorts.kLTrigger.button.whileTrue(intake.runAtVoltageCommand(kIntakeFullVoltage));
//         // Pivoting
//         OperatorPorts.kBackbutton.button.onTrue(intake.setIntakePivotTargetCommand(ZERO));
//         OperatorPorts.kStartbutton.button.onTrue(intake.setIntakePivotTargetCommand(GROUND));
//         // Trash compactor
//         // OperatorPorts.kYbutton.button.whileTrue(new IntakeCompactCommand(intake, GROUND.rotations, COMPACTING.rotations));
//     }
// }
