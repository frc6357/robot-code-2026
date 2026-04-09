package frc.robot.bindings;

import frc.lib.bindings.CommandBinder;
// Imports from robot
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.subsystems.indexer.SK26Indexer;
import static frc.robot.Konstants.IndexerConstants.kIndexerFullVoltage;
import static frc.robot.Ports.OperatorPorts.kLTrigger;
import static frc.robot.Ports.Sensors.launcherSensor;

// Imports from Java/WPILib
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;

public class SK26IndexerBinder implements CommandBinder
{
    Optional<SK26Indexer> indexerSubsystem;
    SK26Indexer indexer;

    Pref<Double> manualIndexerVoltage = SKPreferences.attach("Indexer/ManualVoltage", kIndexerFullVoltage);

    Trigger IndexFeed;
    Trigger IsIdle;
    Trigger IndexBackwards;

    Trigger BallPresent;
    Trigger BallDetected;

    public SK26IndexerBinder(Optional<SK26Indexer> indexerSubsystem)
    {
        this.indexerSubsystem = indexerSubsystem;
        if(indexerSubsystem.isEmpty()) {
            return;
        }
        indexer = indexerSubsystem.get();

        // Feed when any shooting state is READY (launcher up to speed)
        IndexFeed = StateHandler.whenCurrentStateReady(MacroState.SCORING)
            .or(StateHandler.whenCurrentStateReady(MacroState.SHUTTLING))
            .or(StateHandler.whenCurrentStateReady(MacroState.STEADY_STREAM_SCORING))
            .or(StateHandler.whenCurrentStateReady(MacroState.STEADY_STREAM_SHUTTLING));

        IndexBackwards = StateHandler.whenCurrentStateWaiting(MacroState.SCORING)
            .or(StateHandler.whenCurrentStateWaiting(MacroState.SHUTTLING))
            .or(StateHandler.whenCurrentStateWaiting(MacroState.STEADY_STREAM_SCORING))
            .or(StateHandler.whenCurrentStateWaiting(MacroState.STEADY_STREAM_SHUTTLING));

        
        // For simple trigger bindings (if necessary)
        IsIdle = StateHandler.whenCurrentState(MacroState.IDLE);

        BallPresent = new Trigger(() -> indexerSubsystem.get().isBallPresent());

        BallDetected = new Trigger(() -> launcherSensor.getIsDetected().getValue());
    }

    public Command IndexerUnjam() {
        if(indexerSubsystem.get().getUnjamTimerTime()>.5) {
            return Commands.race(
                Commands.defer(() -> indexer.feedCommand(() -> -manualIndexerVoltage.get()), Set.of(indexer)),
                Commands.waitSeconds(0.5)
            );
        } else {
            return Commands.race(
                Commands.defer(() -> indexer.feedCommand(() -> manualIndexerVoltage.get()), Set.of(indexer)),
                Commands.waitSeconds(0.5)
            );
        }
    }

    public void bindButtons()
    {
        if (indexerSubsystem.isEmpty())
        {
            return;
        }

        // IndexFeed.whileTrue(indexer.feedCommand(kIndexerFullVoltage));

        // kRTrigger.button.whileTrue(Commands.defer(() -> indexer.feedCommand(() -> manualIndexerVoltage.get()), Set.of(indexer)));

        // IndexFeed.whileTrue(Commands.repeatingSequence(
        //     Commands.race(
        //         Commands.defer(() -> indexer.feedCommand(() -> manualIndexerVoltage.get()), Set.of(indexer)),
        //         Commands.waitSeconds(1.5)
        //     ),
        //     Commands.race(
        //         Commands.defer(() -> indexer.feedCommand(() -> -manualIndexerVoltage.get()), Set.of(indexer)),
        //         Commands.waitSeconds(0.5)
        //     )
        // ).withName("IndexerFeedAndUnjam"));

        IndexFeed.whileTrue(Commands.repeatingSequence(
            Commands.race(
                Commands.defer(() -> indexer.feedCommand(() -> manualIndexerVoltage.get()), Set.of(indexer)),
                Commands.waitSeconds(1.5)
            ),
            IndexerUnjam()
        ).withName("IndexerFeedAndUnjam"));
        // IndexFeed.debounce(0.2, DebounceType.kFalling).whileTrue(Commands.repeatingSequence(
        //     Commands.race(
        //         Commands.defer(() -> indexer.feedCommand(() -> manualIndexerVoltage.get()), Set.of(indexer)),
        //         Commands.waitSeconds(3.0)
        //     ),
        //     Commands.race(
        //         Commands.defer(() -> indexer.feedCommand(() -> -manualIndexerVoltage.get()), Set.of(indexer)),
        //         Commands.waitSeconds(0.2)
        //     )
        // ).withName("IndexerFeedAndUnjam"));
        // IndexBackwards.debounce(0.2, DebounceType.kRising).whileTrue(indexer.feedCommand(() -> -manualIndexerVoltage.get()).withName("IndexerWaitingUnjam"));
        
        //IndexFeed.whileTrue(indexer.feedCommand(() -> manualIndexerVoltage.get()));
        
        //IndexFeed.negate().whileTrue(Commands.defer(() -> indexer.feedCommand(() -> {return -manualIndexerVoltage.get() / 8.0;}), Set.of(indexer)));
        kLTrigger.button.onTrue(Commands.defer(() -> indexer.feedCommand(() -> -manualIndexerVoltage.get()), Set.of(indexer)));
        kLTrigger.button.onFalse(Commands.defer(
                    () -> IndexFeed.getAsBoolean()
                        ? indexer.feedCommand(() -> manualIndexerVoltage.get())
                        : indexer.idleIndexerCommand(),
                    Set.of(indexer)));
        
        // Removed IndexIdle binding - it was using an uninitialized trigger
        // If you need idle behavior, set it as the default command instead
    }
}