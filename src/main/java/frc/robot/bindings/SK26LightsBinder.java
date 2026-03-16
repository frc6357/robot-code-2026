package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.RobotContainer;
import frc.robot.StateHandler;
import frc.robot.StateHandler.MacroState;
import frc.robot.subsystems.lights.LightMode;
import frc.robot.subsystems.lights.SK26Lights;

public class SK26LightsBinder implements CommandBinder {
    private final Optional<SK26Lights> lightsSubsystem;
    private SK26Lights lights; // Only valid if lightsSubsystem is present

    private Trigger auto;
    private Trigger teleop;
    private Trigger disabledFMS;
    private Trigger disabledNoFMS;
    private Trigger shuttlingWaiting;
    private Trigger shuttlingReady;
    private Trigger scoringWaiting;
    private Trigger scoringReady;
    private Trigger steadyStreamShuttlingWaiting;
    private Trigger steadyStreamShuttlingReady;
    private Trigger steadyStreamScoringWaiting;
    private Trigger steadyStreamScoringReady;
    private Trigger intakeWaiting;
    private Trigger intakeReady;

    boolean lightsPresent = false;

    public SK26LightsBinder(Optional<SK26Lights> lightsSubsystem) {
        this.lightsSubsystem = lightsSubsystem;
        shuttlingWaiting = StateHandler.whenCurrentStateWaiting(MacroState.SHUTTLING);
        steadyStreamShuttlingWaiting = StateHandler.whenCurrentStateWaiting(MacroState.STEADY_STREAM_SHUTTLING);
        shuttlingReady = StateHandler.whenCurrentStateReady(MacroState.SHUTTLING);
        steadyStreamShuttlingReady = StateHandler.whenCurrentStateReady(MacroState.STEADY_STREAM_SHUTTLING);
        scoringWaiting = StateHandler.whenCurrentStateWaiting(MacroState.SCORING);
        steadyStreamScoringWaiting = StateHandler.whenCurrentStateWaiting(MacroState.STEADY_STREAM_SCORING);
        scoringReady = StateHandler.whenCurrentStateReady(MacroState.SCORING);
        steadyStreamScoringReady = StateHandler.whenCurrentStateReady(MacroState.STEADY_STREAM_SCORING);
        intakeWaiting = StateHandler.whenCurrentStateWaiting(MacroState.INTAKING);
        intakeReady = StateHandler.whenCurrentStateReady(MacroState.INTAKING);
        auto = new Trigger(DriverStation::isAutonomousEnabled);
        teleop = new Trigger(DriverStation::isTeleopEnabled);
        disabledFMS = new Trigger(
            () -> DriverStation.isDisabled()
               && (DriverStation.isFMSAttached() || DriverStation.isDSAttached()));
        disabledNoFMS = new Trigger(
            () -> DriverStation.isDisabled()
               && !DriverStation.isFMSAttached()
               && !DriverStation.isDSAttached());
    }

    @Override
    public void bindButtons() {
        if (lightsSubsystem.isEmpty()) {
            return;
        }
        lights = lightsSubsystem.get();
        lightsPresent = true;

        // ── Game-state triggers ───────────────────────────────────────────────────

        // Autonomous — rainbow party mode
        auto.onTrue(lights.setMode(LightMode.RAINBOW, "Auto (Party)"))
            .onFalse(handleEffectFallbackCommand);

        // Main teleop (not in endgame window)
        teleop.onTrue(lights.setMode(LightMode.ALLIANCE_GRADIENT, "Teleop (Alliance)"))
            .onFalse(handleEffectFallbackCommand);

        // Disabled with DS/FMS connected
        disabledFMS.onTrue(lights.setMode(LightMode.SKBLUE_GRADIENT, "Disabled (DS Connected)"))
            .onFalse(handleEffectFallbackCommand);

        // Disabled with no DS
        disabledNoFMS.onTrue(lights.setMode(LightMode.BREATHING_SKBLUE, "Disabled (No DS)"))
            .onFalse(handleEffectFallbackCommand);

        // ── Macro-state-based lights ──────────────────────────────────────────────

        /* Shuttling = Yellow */
        shuttlingWaiting.onTrue(
            lights.setMode(LightMode.SOLID_YELLOW, "Shuttling (Waiting)")
        ).onFalse(handleEffectFallbackCommand);
        shuttlingReady.onTrue(
            lights.setMode(LightMode.STROBE_YELLOW, "Shuttling (Ready)")
        ).onFalse(handleEffectFallbackCommand);

        /* Scoring = SK Blue */
        scoringWaiting.onTrue(
            lights.setMode(LightMode.SOLID_GREEN, "Scoring (Waiting)")
        ).onFalse(handleEffectFallbackCommand);

        scoringReady.onTrue(
            lights.setMode(LightMode.STROBE_GREEN, "Scoring (Ready)")
        ).onFalse(handleEffectFallbackCommand);

        /* Intaking = White */
        intakeWaiting.onTrue(
            lights.setMode(LightMode.SOLID_WHITE, "Intaking (Waiting)")
        ).onFalse(handleEffectFallbackCommand);

        intakeReady.onTrue(
            lights.setMode(LightMode.STROBE_WHITE, "Intaking (Ready)")
        ).onFalse(handleEffectFallbackCommand);

        /* Steady Stream Scoring = Dual White/Green */
        steadyStreamScoringWaiting.onTrue(
            lights.setMode(LightMode.DUAL_SOLID_WHITE_GREEN, "Steady Stream Scoring (Waiting)")
        ).onFalse(handleEffectFallbackCommand);

        steadyStreamScoringReady.onTrue(
            lights.setMode(LightMode.DUAL_STROBE_WHITE_GREEN, "Steady Stream Scoring (Ready)")
        ).onFalse(handleEffectFallbackCommand);

        /* Steady Stream Shuttling = Dual White/Yellow */
        steadyStreamShuttlingWaiting.onTrue(
            lights.setMode(LightMode.DUAL_SOLID_WHITE_YELLOW, "Steady Stream Shuttling (Waiting)")
        ).onFalse(handleEffectFallbackCommand);

        steadyStreamShuttlingReady.onTrue(
            lights.setMode(LightMode.DUAL_STROBE_WHITE_YELLOW, "Steady Stream Shuttling (Ready)")
        ).onFalse(handleEffectFallbackCommand);

        /* Shift ending soon = Orange */
        RobotContainer.shiftEndingSoon.onTrue(
            lights.setMode(LightMode.STROBE_ORANGE, "Shift Ending Soon")
        )
        .onFalse(handleEffectFallbackCommand);
    }

    private Command handleEffectFallbackCommand = Commands.runOnce(this::handleEffectFallback).ignoringDisable(true);

    /**
     * Used whenever a high-priority effect (like state-based or shift ending soon)
     * needs to temporarily override the current lights mode, but we want to return
     * to the correct mode after the effect ends instead of just going back to the
     * default for the current game state. 
     */
    private void handleEffectFallback() {
        if(!lightsPresent) {
            return;
        }

        // Really disguting chain of if-elses, but it works and it's only called when an 
        // effect ends so it won't cause any performance issues. 

        if(scoringReady.getAsBoolean()) {
            CommandScheduler.getInstance().schedule(lights.setMode(LightMode.STROBE_GREEN));
        }
        else if(scoringWaiting.getAsBoolean()) {
            CommandScheduler.getInstance().schedule(lights.setMode(LightMode.SOLID_GREEN));
        }
        else if(shuttlingReady.getAsBoolean()) {
            CommandScheduler.getInstance().schedule(lights.setMode(LightMode.STROBE_YELLOW));
        }
        else if(shuttlingWaiting.getAsBoolean()) {
            CommandScheduler.getInstance().schedule(lights.setMode(LightMode.SOLID_YELLOW));
        }
        else if(intakeReady.getAsBoolean()) {
            CommandScheduler.getInstance().schedule(lights.setMode(LightMode.STROBE_WHITE));
        }
        else if(intakeWaiting.getAsBoolean()) {
            CommandScheduler.getInstance().schedule(lights.setMode(LightMode.SOLID_WHITE));
        }
        else if(steadyStreamScoringReady.getAsBoolean()) {
            CommandScheduler.getInstance().schedule(lights.setMode(LightMode.DUAL_STROBE_WHITE_GREEN));
        }
        else if(steadyStreamScoringWaiting.getAsBoolean()) {
            CommandScheduler.getInstance().schedule(lights.setMode(LightMode.DUAL_SOLID_WHITE_GREEN));
        }
        else if(steadyStreamShuttlingReady.getAsBoolean()) {
            CommandScheduler.getInstance().schedule(lights.setMode(LightMode.DUAL_STROBE_WHITE_YELLOW));
        }
        else if(steadyStreamShuttlingWaiting.getAsBoolean()) {
            CommandScheduler.getInstance().schedule(lights.setMode(LightMode.DUAL_SOLID_WHITE_YELLOW));
        }
        else if(teleop.getAsBoolean()) {
            CommandScheduler.getInstance().schedule(lights.setMode(LightMode.ALLIANCE_GRADIENT));
        }
        else if(auto.getAsBoolean()) {
            CommandScheduler.getInstance().schedule(lights.setMode(LightMode.RAINBOW));
        }
        else if(disabledFMS.getAsBoolean()) {
            CommandScheduler.getInstance().schedule(lights.setMode(LightMode.SKBLUE_GRADIENT));
        }
        else if(disabledNoFMS.getAsBoolean()) {
            CommandScheduler.getInstance().schedule(lights.setMode(LightMode.BREATHING_SKBLUE));
        }
    }
}