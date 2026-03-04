package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.launcher.SK26Feeder;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.subsystems.launcher.BangBangLauncher;

public class SK26FeederBinder implements CommandBinder {

    Optional<SK26Feeder> feederSubsystem;

    Trigger launcherRunning;

    public SK26FeederBinder(Optional<SK26Feeder> feederSubsystem) {
        this.feederSubsystem = feederSubsystem;
        launcherRunning = StateHandler.whenCurrentState(MacroState.SCORING)
                .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING))
                .or(StateHandler.whenCurrentState(MacroState.SHUTTLING))
                .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING));
    }

    @Override
    public void bindButtons() {
        if(feederSubsystem.isPresent()) {
            SK26Feeder feeder = feederSubsystem.get();

            // launcherRunning.whileTrue(new Command(() -> {
            //     feeder.runFeeder();
            // }, feeder).onFalse(new Command(() -> {
            //     feeder.stopFeeder();
            // }, feeder)));
        }
    }
    
}
