package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kLTrigger;
import static frc.robot.Ports.OperatorPorts.kRTrigger;
import static frc.robot.Konstants.FeederConstants.kFeederRunningVoltage;

import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.feeder.SK26Feeder;
import frc.robot.subsystems.launcher.mechanisms.BangBangLauncher;
import frc.robot.subsystems.launcher.mechanisms.SK26DualLauncher;
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

    Trigger runForwards;
    Trigger runBackwards;

    Trigger idle;

    Trigger scoring;

    /** Operator RT manual launch, gated on the flywheel actually being up to speed. */
    Trigger manualLaunch = new Trigger(() -> false);

    /**
     * Binds the feeder subsystem to state-driven triggers.
     *
     * @param feederSubsystem optional feeder subsystem instance (kept optional for project wiring)
     * @param launcherSubsystem optional dual launcher, used to gate the manual launch feed on the
     *                          flywheel reaching speed
     */
    public SK26FeederBinder(Optional<SK26Feeder> feederSubsystem, Optional<SK26DualLauncher> launcherSubsystem) {
        this.feederSubsystem = feederSubsystem;

        // Hold operator RT to feed manually, but only once the flywheel is at speed so fuel is
        // never pushed into a spinning-up launcher. The falling debounce keeps the feed running
        // through the RPM dip each ball causes, instead of stuttering on/off.
        launcherSubsystem.ifPresent(launcher ->
            manualLaunch = kRTrigger.button.and(
                new Trigger(launcher::atTargetVelocity).debounce(0.5, DebounceType.kFalling)));
        // this.launcherRunningState = StateHandler.whenCurrentState(MacroState.SCORING)
        //         .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING))
        //         .or(StateHandler.whenCurrentState(MacroState.SHUTTLING))
        //         .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING));
        this.runForwards = StateHandler.whenCurrentStateReady(MacroState.SCORING)
            .or(StateHandler.whenCurrentStateReady(MacroState.SHUTTLING))
            .or(StateHandler.whenCurrentStateReady(MacroState.STEADY_STREAM_SCORING))
            .or(StateHandler.whenCurrentStateReady(MacroState.STEADY_STREAM_SHUTTLING));

        this.runBackwards = StateHandler.whenCurrentStateWaiting(MacroState.SCORING)
            .or(StateHandler.whenCurrentStateWaiting(MacroState.SHUTTLING))
            .or(StateHandler.whenCurrentStateWaiting(MacroState.STEADY_STREAM_SCORING))
            .or(StateHandler.whenCurrentStateWaiting(MacroState.STEADY_STREAM_SHUTTLING))
            .or(StateHandler.whenCurrentStateWaiting(MacroState.CLIMB_AND_SCORE));
        
        idle = StateHandler.whenCurrentState(MacroState.IDLE);

        scoring = StateHandler.whenCurrentState(MacroState.SCORING)
            .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING));
    }

    @Override
    public void bindButtons() {
        if(feederSubsystem.isPresent()) 
        {
            SK26Feeder feeder = feederSubsystem.get();
            //runFeederFromState.whileTrue(new FeederFeedCommand(feeder, kFeederRunningVelocity));
            // kRTrigger.button.whileTrue(Commands.defer(() -> feeder.feedCommand(() -> manualFeederVoltage.get()), Set.of(feeder)));
            // kBbutton.button.whileTrue(Commands.defer(() -> feeder.feedCommand(() -> -manualFeederVoltage.get()), Set.of(feeder)));

            // runForwards.debounce(0.2, DebounceType.kFalling).whileTrue(Commands.repeatingSequence(
            //     Commands.race(
            //         Commands.defer(() -> feeder.feedCommand(() -> manualFeederVoltage.get()), Set.of(feeder)),
            //         Commands.waitSeconds(1.5)
            //     )
            // ).withName("FeederFeedAndUnjam"));
            // runBackwards.debounce(0.2, DebounceType.kRising).whileTrue(feeder.feedCommand(() -> manualFeederVoltage.get()).withName("FeederWaiting"));
            runForwards.or(runBackwards).or(manualLaunch).whileTrue(Commands.defer(() -> feeder.feedCommand(() -> manualFeederVoltage.get()), Set.of(feeder)).withName("FeederRun"));
            kLTrigger.button.onTrue(Commands.defer(() -> feeder.feedCommand(() -> -manualFeederVoltage.get()), Set.of(feeder)));
            kLTrigger.button.onFalse(Commands.defer(
                    () -> runForwards.getAsBoolean() || manualLaunch.getAsBoolean()
                        ? feeder.feedCommand(() -> manualFeederVoltage.get())
                        : feeder.idleFeederCommand(),
                    Set.of(feeder)));

            // scoring.onTrue(feeder.startBPSTimer());
            // scoring.onFalse(feeder.stopBPSTimer());

            // runFeederFromState.whileTrue(feeder.feedCommand(kFeederRunningVoltage));
            // runLowVoltage.whileTrue(feeder.feedCommand(kFeederWaitingVoltage));
            // idle.whileTrue(feeder.idleFeederCommand());
        }
    }
    
}
