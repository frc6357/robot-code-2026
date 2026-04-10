package frc.robot.bindings;

import java.util.Optional;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bindings.CommandBinder;
import frc.robot.StateHandler;
import frc.robot.Ports.DriverPorts;
import frc.robot.StateHandler.MacroState;
import frc.robot.StateHandler.MacroState.Status;

public class SK26StateBinder implements CommandBinder {
    private Optional<StateHandler> stateHandlerContainer;
    private StateHandler stateHandler;

    Trigger turnOffLaunch;
    Trigger turnOnScoring;
    Trigger turnOnShuttling;
    Trigger turnOnSpitting;

    Trigger launcherReadyToScore;
    Trigger launcherReadyToShuttle;
    Trigger intakeDeployed;
    Trigger intakeAvoidingMajorFouls;
    Trigger turretReadyToScore;
    Trigger turretReadyToShuttle;
    Trigger inAllianceZone;
    Trigger outOfAllianceZone;
    Trigger notNearTower;
    Trigger climbReady;
    Trigger climbStowed;

    // Trigger that is true when launcher is in an active launching state
    Trigger launcherActive;
    // Debouncer to ensure launcher has been active for 0.8 seconds before allowing turn off
    Debouncer launcherActiveDebouncer = new Debouncer(0.8, DebounceType.kRising);

    public SK26StateBinder(Optional<StateHandler> stateHandlerContainer) {
        this.stateHandlerContainer = stateHandlerContainer;
        this.stateHandler = stateHandlerContainer.orElse(null);

        // Trigger definitions:
        /* Subsystem States */
        if (stateHandler != null) {
            launcherReadyToScore = stateHandler.getLauncherReadyToScore();
            launcherReadyToShuttle = stateHandler.getLauncherReadyToShuttle();
            intakeDeployed = stateHandler.getIntakeDeployed();
            intakeAvoidingMajorFouls = stateHandler.getIntakeAvoidingMajorFouls();
            turretReadyToScore = stateHandler.getTurretReadyToScore();
            turretReadyToShuttle = stateHandler.getTurretReadyToShuttle();
            inAllianceZone = stateHandler.getInAllianceZone();
            outOfAllianceZone = inAllianceZone.negate();
            notNearTower = stateHandler.getNotNearTower();
            climbReady = stateHandler.getClimbReady();
            climbStowed = stateHandler.getClimbStowed();
        }

        /* Buttons */
        turnOnScoring = DriverPorts.kRTrigger.button.and(inAllianceZone);
        turnOnShuttling = DriverPorts.kRTrigger.button.and(outOfAllianceZone);
        turnOnSpitting = DriverPorts.kXbutton.button;

        if (stateHandler != null) {
            // Create trigger for when launcher is in any active launching state
            launcherActive = new Trigger(() -> {
                MacroState state = stateHandler.getCurrentState();
                return state == MacroState.SCORING 
                    || state == MacroState.SHUTTLING
                    || state == MacroState.STEADY_STREAM_SCORING 
                    || state == MacroState.STEADY_STREAM_SHUTTLING;
            });

            // Turn off trigger requires: double-press AND launcher has been active for 0.8s
            Trigger launcherActiveFor800ms = new Trigger(() -> launcherActiveDebouncer.calculate(launcherActive.getAsBoolean()));
            turnOffLaunch = DriverPorts.kRTrigger.button.multiPress(2, 0.33).and(launcherActiveFor800ms);
        } else {
            turnOffLaunch = DriverPorts.kRTrigger.button.multiPress(2, 0.33);
        }
    }

    @Override
    public void bindButtons() {
        if(stateHandlerContainer.isEmpty()) {
            return;
        }
        bindRobotStates();
        bindDriverButtons();
        bindOperatorButtons();
    }

    private void bindRobotStates() {
        inAllianceZone.and(notNearTower).and(launcherReadyToScore).and(turretReadyToScore)
            .onTrue(
                stateHandler.setMacroStateStatusCommand(MacroState.SCORING, MacroState.Status.READY)
                .alongWith(stateHandler.setMacroStateStatusCommand(MacroState.STEADY_STREAM_SCORING, Status.READY))
            )
            .onFalse(
                stateHandler.setMacroStateStatusCommand(MacroState.SCORING, MacroState.Status.WAITING)
                .alongWith(stateHandler.setMacroStateStatusCommand(MacroState.STEADY_STREAM_SCORING, Status.WAITING))
            );
        outOfAllianceZone.and(launcherReadyToShuttle).and(turretReadyToShuttle)
            .onTrue(
                stateHandler.setMacroStateStatusCommand(MacroState.SHUTTLING, MacroState.Status.READY)
                .alongWith(stateHandler.setMacroStateStatusCommand(MacroState.STEADY_STREAM_SHUTTLING, Status.READY))
            )
            .onFalse(
                stateHandler.setMacroStateStatusCommand(MacroState.SHUTTLING, MacroState.Status.WAITING)
                .alongWith(stateHandler.setMacroStateStatusCommand(MacroState.STEADY_STREAM_SHUTTLING, Status.WAITING))
            );
        intakeDeployed
            .onTrue(
                stateHandler.setMacroStateStatusCommand(MacroState.INTAKING, Status.READY)
                .alongWith(stateHandler.setMacroStateStatusCommand(MacroState.SPITTING, Status.READY))
            )
            .onFalse(
                stateHandler.setMacroStateStatusCommand(MacroState.INTAKING, Status.WAITING)
                .alongWith(stateHandler.setMacroStateStatusCommand(MacroState.SPITTING, Status.WAITING))
            );
        // intakeAvoidingMajorFouls.and(climbReady).and(inAllianceZone)
        //     .onFalse(stateHandler.setMacroStateStatusCommand(MacroState.CLIMBING, Status.WAITING));
        
        intakeAvoidingMajorFouls.and(climbReady).and(inAllianceZone).and(launcherReadyToScore).and(turretReadyToScore)
            .onTrue(stateHandler.setMacroStateStatusCommand(MacroState.CLIMB_AND_SCORE, Status.READY))
            .onFalse(stateHandler.setMacroStateStatusCommand(MacroState.CLIMB_AND_SCORE, Status.WAITING));
        DriverPorts.kStartbutton.button.onTrue(stateHandler.setMacroStateStatusCommand(MacroState.CLIMBING, MacroState.Status.WAITING))
            .onFalse(stateHandler.setMacroStateStatusCommand(MacroState.CLIMBING, MacroState.Status.WAITING));

    }

    private void bindDriverButtons() 
    {
        DriverPorts.kLTrigger.button.onTrue(stateHandler.addIntakeToRequestedStateCommand())
            .onFalse(stateHandler.removeIntakeFromRequestedStateCommand());
        
        DriverPorts.kStartbutton.button.onTrue(stateHandler.requestStateCommand(MacroState.CLIMBING))
            .onFalse(stateHandler.removeClimbFromRequestedStateCommand());

        turnOnScoring.onTrue(stateHandler.addScoringToRequestedStateCommand());
        turnOnScoring.onFalse(stateHandler.removeScoringFromRequestedStateCommand());

        turnOnShuttling.onTrue(stateHandler.addShuttlingToRequestedStateCommand());
        turnOnShuttling.onFalse(stateHandler.removeShuttlingFromRequestedStateCommand());

        turnOnSpitting.onTrue(stateHandler.requestStateCommand(MacroState.SPITTING));
        turnOnSpitting.onFalse(stateHandler.requestStateCommand(MacroState.IDLE));
    }

    private void bindOperatorButtons() {
        // OperatorPorts.kRTrigger.button.onTrue(stateHandler.addShuttlingToRequestedStateCommand());
        // OperatorPorts.kRTrigger.button.onFalse(stateHandler.removeShuttlingFromRequestedStateCommand());
    }
}
