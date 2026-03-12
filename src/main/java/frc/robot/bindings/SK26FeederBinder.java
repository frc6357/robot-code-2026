package frc.robot.bindings;

import static frc.robot.Ports.OperatorPorts.kAbutton;
import static frc.robot.Ports.OperatorPorts.kDownDpad;
import static frc.robot.Konstants.FeederConstants.kFeederRunningVoltage;
import static frc.robot.Konstants.FeederConstants.kFeederWaitingVoltage;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.feeder.SK26Feeder;
import frc.robot.subsystems.launcher.mechanisms.BangBangLauncher;
import frc.lib.bindings.CommandBinder;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;

public class SK26FeederBinder implements CommandBinder {

    Optional<SK26Feeder> feederSubsystem;
    BangBangLauncher launcher;

    Trigger launcherRunningState;
    Trigger launcherAtSpeed;

    Trigger runFeederFromState;
    Trigger runLowVoltage;

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
        this.runFeederFromState = StateHandler.whenCurrentStateReady(MacroState.SCORING)
            .or(StateHandler.whenCurrentStateReady(MacroState.SHUTTLING))
            .or(StateHandler.whenCurrentStateReady(MacroState.STEADY_STREAM_SCORING))
            .or(StateHandler.whenCurrentStateReady(MacroState.STEADY_STREAM_SHUTTLING));

        this.runLowVoltage = StateHandler.whenCurrentStateWaiting(MacroState.SCORING)
            .or(StateHandler.whenCurrentStateWaiting(MacroState.SHUTTLING))
            .or(StateHandler.whenCurrentStateWaiting(MacroState.STEADY_STREAM_SCORING))
            .or(StateHandler.whenCurrentStateWaiting(MacroState.STEADY_STREAM_SHUTTLING))
            .or(StateHandler.whenCurrentStateWaiting(MacroState.CLIMB_AND_SCORE));
    }

    @Override
    public void bindButtons() {
        if(feederSubsystem.isPresent()) 
        {
            SK26Feeder feeder = feederSubsystem.get();
            //runFeederFromState.whileTrue(new FeederFeedCommand(feeder, kFeederRunningVelocity));
            kDownDpad.button.and(StateHandler.whenCurrentState(MacroState.IDLE)).whileTrue(feeder.feedCommand(kFeederRunningVoltage));
            //runFeederFromState.whileTrue(feeder.feedCommand(kFeederRunningVoltage));
            //runLowVoltage.whileTrue(feeder.feedCommand(kFeederWaitingVoltage));
        }
    }
    
}
