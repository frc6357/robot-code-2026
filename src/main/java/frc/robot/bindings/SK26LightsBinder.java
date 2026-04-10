package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.Ports.DriverPorts;
import frc.robot.Ports.OperatorPorts;
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

        // ── Auto-mode gate (only fire state triggers when chooser is set to AUTO) ─
        Trigger autoMode = new Trigger(() -> lights.isAutoLightsEnabled());

        // ── Game-state triggers ───────────────────────────────────────────────────

        // Autonomous — rainbow party mode
        auto.and(autoMode).onTrue(lights.setMode(LightMode.RAINBOW, "Auto (Party)"))
            .onFalse(handleEffectFallbackCommand);

        // Main teleop (not in endgame window)
        teleop.and(autoMode).onTrue(lights.setMode(LightMode.ALLIANCE_GRADIENT, "Teleop (Alliance)"))
            .onFalse(handleEffectFallbackCommand);

        // Disabled with DS/FMS connected
        disabledFMS.and(autoMode).onTrue(lights.setMode(LightMode.SKBLUE_GRADIENT, "Disabled (DS Connected)"))
            .onFalse(handleEffectFallbackCommand);

        // Disabled with no DS
        disabledNoFMS.and(autoMode).onTrue(lights.setMode(LightMode.BREATHING_SKBLUE, "Disabled (No DS)"))
            .onFalse(handleEffectFallbackCommand);

        // ── Macro-state-based lights ──────────────────────────────────────────────

        /* Shuttling = Yellow */
        shuttlingWaiting.and(autoMode).onTrue(
            lights.setMode(LightMode.SOLID_YELLOW, "Shuttling (Waiting)")
        ).onFalse(handleEffectFallbackCommand);
        shuttlingReady.and(autoMode).onTrue(
            lights.setMode(LightMode.STROBE_YELLOW, "Shuttling (Ready)")
        ).onFalse(handleEffectFallbackCommand);

        /* Scoring = SK Blue */
        scoringWaiting.and(autoMode).onTrue(
            lights.setMode(LightMode.SOLID_WHITE, "Scoring (Waiting)")
        ).onFalse(handleEffectFallbackCommand);

        scoringReady.and(autoMode).onTrue(
            lights.setMode(LightMode.STROBE_WHITE, "Scoring (Ready)")
        ).onFalse(handleEffectFallbackCommand);

        /* Intaking = White */
        intakeWaiting.and(autoMode).onTrue(
            lights.setMode(LightMode.SOLID_GREEN, "Intaking (Waiting)")
        ).onFalse(handleEffectFallbackCommand);

        intakeReady.and(autoMode).onTrue(
            lights.setMode(LightMode.STROBE_GREEN, "Intaking (Ready)")
        ).onFalse(handleEffectFallbackCommand);

        /* Steady Stream Scoring = Dual White/Green */
        steadyStreamScoringWaiting.and(autoMode).onTrue(
            lights.setMode(LightMode.DUAL_SOLID_WHITE_GREEN, "Steady Stream Scoring (Waiting)")
        ).onFalse(handleEffectFallbackCommand);

        steadyStreamScoringReady.and(autoMode).onTrue(
            lights.setMode(LightMode.DUAL_STROBE_WHITE_GREEN, "Steady Stream Scoring (Ready)")
        ).onFalse(handleEffectFallbackCommand);

        /* Steady Stream Shuttling = Dual White/Yellow */
        steadyStreamShuttlingWaiting.and(autoMode).onTrue(
            lights.setMode(LightMode.DUAL_SOLID_WHITE_YELLOW, "Steady Stream Shuttling (Waiting)")
        ).onFalse(handleEffectFallbackCommand);

        steadyStreamShuttlingReady.and(autoMode).onTrue(
            lights.setMode(LightMode.DUAL_STROBE_WHITE_YELLOW, "Steady Stream Shuttling (Ready)")
        ).onFalse(handleEffectFallbackCommand);

        /* Shift ending soon = Orange */
        RobotContainer.shiftEndingSoon.and(autoMode).onTrue(
            lights.setMode(LightMode.STROBE_ORANGE, "Shift Ending Soon")
        )
        .onFalse(handleEffectFallbackCommand);

        RobotContainer.shiftNotice.and(autoMode).onTrue(
            lights.setMode(LightMode.STROBE_PURPLE, "Shift Ending Notice")
        )
        .onFalse(handleEffectFallbackCommand);

        // ── Game bindings (gated by game controller mode) ─────────────────────────

        Trigger gameModeEnabled = new Trigger(
            () -> SmartDashboard.getBoolean("Lights/Game Controller Mode", false));
        Trigger gameAndDisabled = gameModeEnabled.and(() -> DriverStation.isDisabled());

        // -- Simon Says: A=green(1), B=red(0), X=blue(2), Y=yellow(3) --
        Trigger simonMode = new Trigger(() -> lights.getCurrentMode() == LightMode.SIMON_SAYS);
        Trigger simonActive = gameAndDisabled.and(simonMode);

        // Driver ABXY
        simonActive.and(DriverPorts.kAbutton.button)
            .onTrue(lights.simonGreenButton());
        simonActive.and(DriverPorts.kBbutton.button)
            .onTrue(lights.simonRedButton());
        simonActive.and(DriverPorts.kXbutton.button)
            .onTrue(lights.simonBlueButton());
        simonActive.and(DriverPorts.kYbutton.button)
            .onTrue(lights.simonYellowButton());

        // Operator ABXY
        simonActive.and(OperatorPorts.kAbutton.button)
            .onTrue(lights.simonGreenButton());
        simonActive.and(OperatorPorts.kBbutton.button)
            .onTrue(lights.simonRedButton());
        simonActive.and(OperatorPorts.kXbutton.button)
            .onTrue(lights.simonBlueButton());
        simonActive.and(OperatorPorts.kYbutton.button)
            .onTrue(lights.simonYellowButton());

        // -- Tug of War: Driver A = pull left, Operator A = pull right --
        Trigger tugMode = new Trigger(() -> lights.getCurrentMode() == LightMode.TUG_OF_WAR);
        Trigger tugActive = gameAndDisabled.and(tugMode);

        tugActive.and(DriverPorts.kAbutton.button)
            .onTrue(lights.gameButtonPressed());
        tugActive.and(OperatorPorts.kAbutton.button)
            .onTrue(lights.gameButtonPressedAlt());

        // Reset current game with Back button on either controller
        gameAndDisabled.and(DriverPorts.kBackbutton.button)
            .onTrue(lights.resetCurrentGame());
        gameAndDisabled.and(OperatorPorts.kBackbutton.button)
            .onTrue(lights.resetCurrentGame());
    }

    private Command handleEffectFallbackCommand = Commands.runOnce(this::handleEffectFallback).ignoringDisable(true);

    /**
     * Used whenever a high-priority effect (like state-based or shift ending soon)
     * needs to temporarily override the current lights mode, but we want to return
     * to the correct mode after the effect ends instead of just going back to the
     * default for the current game state. 
     */
    private void handleEffectFallback() {
        if(!lightsPresent || !lights.isAutoLightsEnabled()) {
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