package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.launcher.BangBangLauncher;
import frc.robot.subsystems.launcher.SK26Feeder;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;

public class SK26FeederBinder implements CommandBinder {

    Optional<SK26Feeder> feederSubsystem;
    BangBangLauncher launcher;

    Trigger launcherRunningState;
    Trigger launcherAtSpeed;

    /**
     * Binds the feeder subsystem to state-driven triggers.
     *
     * @param feederSubsystem optional feeder subsystem instance (kept optional for project wiring)
     */
    public SK26FeederBinder(Optional<SK26Feeder> feederSubsystem) {
        this.feederSubsystem = feederSubsystem;
        this.launcherRunningState = StateHandler.whenCurrentState(MacroState.SCORING)
                .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SCORING))
                .or(StateHandler.whenCurrentState(MacroState.SHUTTLING))
                .or(StateHandler.whenCurrentState(MacroState.STEADY_STREAM_SHUTTLING));
        this.launcherAtSpeed = new Trigger(launcher::isAtGoal);
    }

    @Override
    public void bindButtons() {
        if(feederSubsystem.isPresent()) {
            SK26Feeder feeder = feederSubsystem.get();

            launcherAtSpeed.whileTrue(new StartEndCommand(feeder::runFeeder, feeder::stopFeeder, feeder));
        }
    }
    
}
