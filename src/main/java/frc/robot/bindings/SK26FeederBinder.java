package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kLTrigger;
import static frc.robot.Konstants.FeederConstants.kFeederRunningVoltage;

import static frc.robot.Ports.TesterPorts.kFeederButton;

import java.util.Optional;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.feeder.SK26Feeder;
import frc.robot.subsystems.launcher.mechanisms.BangBangLauncher;
import frc.lib.bindings.CommandBinder;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;

public class SK26FeederBinder implements CommandBinder {

    Optional<SK26Feeder> feederSubsystem;
    BangBangLauncher launcher;

    Pref<Double> manualFeederVoltage = SKPreferences.attach("Feeder/ManualVoltage", kFeederRunningVoltage);

    Trigger launcherRunningState;
    Trigger launcherAtSpeed;

    Trigger runFeederFromState;
    Trigger runLowVoltage;

    Trigger idle;

    /**
     * Binds the feeder subsystem to state-driven triggers.
     *
     * @param feederSubsystem optional feeder subsystem instance (kept optional for project wiring)
     */
    public SK26FeederBinder(Optional<SK26Feeder> feederSubsystem) {
        this.feederSubsystem = feederSubsystem;
        // this.launcherRunningState = StateHandler.whenCurrentState(MacroState.SCORING)
        //         .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING))
        //         .or(StateHandler.whenCurrentState(MacroState.SHUTTLING))
        //         .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING));
        this.runFeederFromState = StateHandler.whenCurrentState(MacroState.SCORING)
            .or(StateHandler.whenCurrentState(MacroState.SHUTTLING))
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING))
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING));

        this.runLowVoltage = StateHandler.whenCurrentStateWaiting(MacroState.SCORING)
            .or(StateHandler.whenCurrentStateWaiting(MacroState.SHUTTLING))
            .or(StateHandler.whenCurrentStateWaiting(MacroState.STEADY_STREAM_SCORING))
            .or(StateHandler.whenCurrentStateWaiting(MacroState.STEADY_STREAM_SHUTTLING))
            .or(StateHandler.whenCurrentStateWaiting(MacroState.CLIMB_AND_SCORE));
        
        idle = StateHandler.whenCurrentState(MacroState.IDLE);
    }

    @Override
    public void bindButtons() {
        if(feederSubsystem.isPresent()) 
        {
            SK26Feeder feeder = feederSubsystem.get();
            //runFeederFromState.whileTrue(new FeederFeedCommand(feeder, kFeederRunningVelocity));
            // kRTrigger.button.whileTrue(Commands.defer(() -> feeder.feedCommand(() -> manualFeederVoltage.get()), Set.of(feeder)));
            // kBbutton.button.whileTrue(Commands.defer(() -> feeder.feedCommand(() -> -manualFeederVoltage.get()), Set.of(feeder)));

            runFeederFromState.whileTrue(Commands.defer(() -> feeder.feedCommand(() -> manualFeederVoltage.get()), Set.of(feeder)).withName("FeederRollersRun"));
            kLTrigger.button.onTrue(Commands.defer(() -> feeder.feedCommand(() -> -manualFeederVoltage.get()), Set.of(feeder)));
            kLTrigger.button.onFalse(Commands.defer(
                    () -> runFeederFromState.getAsBoolean()
                        ? feeder.feedCommand(() -> manualFeederVoltage.get())
                        : feeder.idleFeederCommand(),
                    Set.of(feeder)));

            kFeederButton.button.onTrue(Commands.defer (() -> feeder.feedCommand(() ->  -manualFeederVoltage.get()), Set.of(feeder)));
            kFeederButton.button.onFalse(Commands.defer(
                    () -> runFeederFromState.getAsBoolean()
                        ? feeder.feedCommand(() -> manualFeederVoltage.get())
                        : feeder.idleFeederCommand(),
                    Set.of(feeder)));
            // runFeederFromState.whileTrue(feeder.feedCommand(kFeederRunningVoltage));
            // runLowVoltage.whileTrue(feeder.feedCommand(kFeederWaitingVoltage));
            // idle.whileTrue(feeder.idleFeederCommand());
        }
    }
    
}
