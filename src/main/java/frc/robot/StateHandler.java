package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.commands.PathPlannerCommands;
import frc.lib.subsystems.PathplannerSubsystem;
import frc.lib.utils.Field;
import frc.lib.utils.FieldConstants.LinesVertical;
import frc.lib.utils.FieldConstants.Tower;
import frc.robot.Konstants.LauncherConstants;
import frc.robot.Konstants.SwerveConstants;
import frc.robot.StateHandler.MacroState.Status;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.launcher.mechanisms.BangBangLauncher;
import frc.robot.subsystems.intake.SK26Intake;
import frc.robot.subsystems.turret.SK26Turret;
import lombok.Getter;

/**
 * A class to handle large-scale robot states (macros) such as launching, intaking, climbing, and idling.
 * Each macro state has an associated status to indicate its current condition.
 * A MacroState can still have its status updated while it is not the current nor desired state.
 */
public class StateHandler extends SubsystemBase implements PathplannerSubsystem{
    
    private static final MacroState[] MACRO_STATES = MacroState.values();

    public enum MacroState {
        IDLE(Status.READY),
        SCORING(Status.WAITING),
        SHUTTLING(Status.WAITING),
        INTAKING(Status.WAITING),
        CLIMBING(Status.WAITING),
        STEADY_STREAM_SCORING(Status.WAITING),
        STEADY_STREAM_SHUTTLING(Status.WAITING),
        CLIMB_AND_SCORE(Status.WAITING);
        
        private MacroState(Status status) {
            this.status = status;
        }
        
        private Status status;

        public Status getStatus() {
            return status;
        }
        public void setStatus(Status status) {
            this.status = status;
        }
        
        // Similar to WPILib Command structure
        public enum Status {
            WAITING,
            READY
        }
    }
    public Command setMacroStateStatusCommand(MacroState state, MacroState.Status status) {
        return runOnce(() -> setStatusOf(state, status)).withName("Set" + state.name() + "StatusTo" + status.name());
    }
    public Command setMacroStatesStatusCommand(MacroState[] states, MacroState.Status status) {
        return runOnce(() -> {
            for (MacroState state : states) {
                setStatusOf(state, status);
            }
        }).withName("SetMultipleStatesStatusTo" + status.name());
    }

    private LoggedDashboardChooser<MacroState> stateChooser = new LoggedDashboardChooser<>("State Chooser");        

    private static MacroState currentState = MacroState.IDLE;
    private static MacroState requestedState = MacroState.IDLE;

    // private MacroState previousChosenState = MacroState.IDLE;

    /**
     * Trigger that is true when the launcher is at its target velocity (or no launcher is present).
     * Defaults to true; remapped to the launcher's atGoal when {@link #setLauncherSubsystem} is called.
     */
    @Getter
    private Trigger launcherReady = new Trigger(() -> true);

    /**
     * Trigger that is true when the turret is at its target angle (or no turret is present).
     * Defaults to true; remapped to the turret's atTarget when {@link #setTurretSubsystem} is called.
     */
    @Getter
    private Trigger turretReady = new Trigger(() -> true);

    /**
     * Trigger that is true when the intake positioner is at its target (or no intake is present).
     * Defaults to true; remapped to the intake's isPositionerAtTarget when {@link #setIntakeSubsystem} is called.
     */
    @Getter
    private Trigger intakeReady = new Trigger(() -> true);

    /**
     * Trigger that is true when the robot's chassis is within its own alliance zone.
     * Defaults to false; remapped when a drive subsystem is provided via {@link #setDriveSubsystem}.
     *
     * <p>The check uses the robot's center pose and adds half the chassis length as a buffer,
     * so the trigger fires when the chassis edge is roughly inside the alliance zone line.
     */
    @Getter
    private Trigger inAllianceZone = new Trigger(() -> false);

    /**
     * Trigger that is true when the robot's chassis is NOT within its own alliance zone.
     * Always the logical inverse of {@link #inAllianceZone}.
     */
    @Getter
    private Trigger outOfAllianceZone = inAllianceZone.negate();

    /**
     * Trigger that is true when the shooter is NOT within 10 inches of the tower structure.
     * Defaults to true (safe); remapped when a drive subsystem is provided via {@link #setDriveSubsystem}.
     *
     * <p>The shooter's field-space position is computed from the robot pose plus the
     * robot-to-shooter transform. The tower area is the bounding rectangle defined by
     * its front face, depth, and width, expanded by 10 inches on all sides.
     */
    @Getter
    private Trigger notNearTower = new Trigger(() -> true);

    public StateHandler() {
        // Reset all states to default on construction
        for (MacroState state : MACRO_STATES) {
            state.setStatus(state == MacroState.IDLE ? MacroState.Status.READY : MacroState.Status.WAITING);
        }

        stateChooser.addDefaultOption("IDLE", MacroState.IDLE);
        stateChooser.addOption("SCORING", MacroState.SCORING);
        stateChooser.addOption("INTAKING", MacroState.INTAKING);
        stateChooser.addOption("SHUTTLING", MacroState.SHUTTLING);
        stateChooser.addOption("CLIMBING", MacroState.CLIMBING);
        stateChooser.addOption("SS_SCORING", MacroState.STEADY_STREAM_SCORING);
        stateChooser.addOption("SS_SHUTTLING", MacroState.STEADY_STREAM_SHUTTLING);
        stateChooser.addOption("CLIMB_AND_SCORE", MacroState.CLIMB_AND_SCORE);

        stateChooser.onChange((state) -> this.requestState(state));

        addPathPlannerCommands();
    }

    /**
     * Sets the launcher subsystem reference for checking launcher readiness.
     * If the Optional is present, remaps the {@link #launcherReady} trigger to the
     * launcher's {@code isAtGoal()} method. If empty, leaves the trigger unchanged
     * (defaults to always true).
     *
     * @param launcher Optional containing the BangBangLauncher, or empty if not present
     */
    public void setLauncherSubsystem(Optional<BangBangLauncher> launcher) {
        if (launcher.isEmpty()) {
            return;
        }
        launcherReady = new Trigger(launcher.get()::isAtGoal);
    }

    /**
     * Sets the turret subsystem reference for checking turret readiness.
     * If the Optional is present, remaps the {@link #turretReady} trigger to the
     * turret's {@code atTarget()} method. If empty, leaves the trigger unchanged
     * (defaults to always true).
     *
     * @param turret Optional containing the SK26Turret, or empty if not present
     */
    public void setTurretSubsystem(Optional<SK26Turret> turret) {
        if (turret.isEmpty()) {
            return;
        }
        turretReady = new Trigger(turret.get()::atTarget);
    }

    /**
     * Sets the intake subsystem reference for checking intake readiness.
     * If the Optional is present, remaps the {@link #intakeReady} trigger to the
     * intake's {@code isPositionerAtTarget()} method. If empty, leaves the trigger unchanged
     * (defaults to always true).
     *
     * @param intake Optional containing the SK26Intake, or empty if not present
     */
    public void setIntakeSubsystem(Optional<SK26Intake> intake) {
        if (intake.isEmpty()) {
            return;
        }
        intakeReady = new Trigger(intake.get()::isPositionerAtTarget);
    }

    /**
     * Sets the drive subsystem reference for zone-based triggers.
     * If the Optional is present, remaps {@link #inAllianceZone}, {@link #outOfAllianceZone},
     * and {@link #notNearTower} to evaluate the robot's pose against field boundaries.
     * If empty, leaves the triggers unchanged at their defaults.
     *
     * @param drive Optional containing the SKSwerve, or empty if not present
     */
    public void setDriveSubsystem(Optional<SKSwerve> drive) {
        if (drive.isEmpty()) {
            return;
        }
        SKSwerve swerve = drive.get();
        inAllianceZone = new Trigger(() -> {
            double robotX = swerve.getRobotPose().getX();
            if (Field.isBlue()) {
                return robotX < LinesVertical.allianceZone + Units.inchesToMeters(SwerveConstants.kChassisLength / 2.0);
            } else {
                return robotX > LinesVertical.redAllianceZone - Units.inchesToMeters(SwerveConstants.kChassisLength / 2.0);
            }
        });
        outOfAllianceZone = inAllianceZone.negate();

        notNearTower = new Trigger(() -> {
            // Compute shooter position in field space from robot pose + robot-to-shooter transform
            Translation2d shooterPos = new Pose3d(swerve.getRobotPose())
                .plus(LauncherConstants.kRobotToShooter)
                .toPose2d()
                .getTranslation();

            // Pick the correct tower center based on alliance
            Translation2d towerCenter = Field.isBlue() ? Tower.centerPoint : Tower.redCenterPoint;

            // Tower bounding box expanded by 10 inches on all sides
            double buffer = Units.inchesToMeters(10.0);
            double halfWidth = Tower.width / 2.0 + buffer;
            double halfDepth = Tower.depth / 2.0 + buffer;

            // Tower center X is at the front face; the tower extends backward (toward the wall)
            // Blue: extends from frontFaceX toward 0; Red: extends from (fieldLength - frontFaceX) toward fieldLength
            double towerCenterX = Field.isBlue()
                ? Tower.frontFaceX - Tower.depth / 2.0
                : towerCenter.getX() + Tower.depth / 2.0;
            double towerCenterY = towerCenter.getY();

            // Check if shooter is inside the expanded bounding box
            boolean insideX = Math.abs(shooterPos.getX() - towerCenterX) < halfDepth;
            boolean insideY = Math.abs(shooterPos.getY() - towerCenterY) < halfWidth;

            return !(insideX && insideY);
        });
    }

    @Override
    public void periodic() {
        // Handle state transition
        if (requestedState != currentState) {
            currentState = requestedState;
        }

        // MacroState chosen = stateChooser.get();
        // if (chosen != previousChosenState) {
        //     setCurrentState(chosen);
        //     previousChosenState = chosen;
        // }

        logOutputs();
    }

    private void logOutputs() {
        Logger.recordOutput("StateHandler/Current State", getCurrentState().name());
        Logger.recordOutput("StateHandler/Current State Status", getCurrentState().getStatus().name());
        Logger.recordOutput("StateHandler/Requested State", getRequestedState().name());
        for (MacroState state : MACRO_STATES) {
            Logger.recordOutput("StateHandler/" + state.name() + " Status", state.getStatus().name());
        }
    }

    /**
     * Gets the current state.
     * @return The current MacroState.
     */
    public MacroState getCurrentState() {
        return currentState;
    }
    
    /**
     * Forcefully sets the current state and clears the desired state.
     * @param state The new current MacroState.
     */
    public void setCurrentState(MacroState state) {
        currentState = state;
        clearRequestedState();
    }

    public Command setCurrentStateCommand(MacroState state) {
        return runOnce(() -> setCurrentState(state)).withName("Force" + state.name());
    }

    /**
     * Gets the desired state.
     * @return The desired MacroState.
     */
    public MacroState getRequestedState() {
        return requestedState;
    }

    /**
     * Sets the desired state.
     * @param state The desired MacroState.
     */
    public void requestState(MacroState state) {
        requestedState = state;
    }

    public Command requestStateCommand(MacroState state) {
        return runOnce(() -> requestState(state)).withName("Request" + state.name());
    }

    /**
     * Clears the desired MacroState, setting it to whatever the current state is.
     */
    public void clearRequestedState() {
        requestedState = currentState;
    }

    public Command clearRequestedStateCommand() {
        return runOnce(this::clearRequestedState).withName("ClearRequestedState");
    }

    public MacroState.Status getStatusOf(MacroState state) {
        return state.getStatus();
    }
    
    public void setStatusOf(MacroState state, MacroState.Status status) {
        state.setStatus(status);
    }

    public void removeIntakeFromRequestedState() {
        if(requestedState == MacroState.INTAKING) {
            requestedState = MacroState.IDLE;
        }
        if(requestedState == MacroState.STEADY_STREAM_SCORING) {
            requestedState = MacroState.SCORING;
        }
        if(requestedState == MacroState.STEADY_STREAM_SHUTTLING) {
            requestedState = MacroState.SHUTTLING;
        }
    }

    public Command removeIntakeFromRequestedStateCommand() {
        return runOnce(this::removeIntakeFromRequestedState).withName("RemoveIntakeFromRequestedState");
    }

    public void addIntakeToRequestedState() {
        if(requestedState == MacroState.IDLE) {
            requestedState = MacroState.INTAKING;
        }
        if(requestedState == MacroState.SCORING) {
            requestedState = MacroState.STEADY_STREAM_SCORING;
        }
        if(requestedState == MacroState.SHUTTLING) {
            requestedState = MacroState.STEADY_STREAM_SHUTTLING;
        }
    }

    public Command addIntakeToRequestedStateCommand() {
        return runOnce(this::addIntakeToRequestedState).withName("AddIntakeToRequestedState");
    }

    public void toggleIntakeInRequestedState() {
        if(requestedState == MacroState.INTAKING || requestedState == MacroState.STEADY_STREAM_SCORING || requestedState == MacroState.STEADY_STREAM_SHUTTLING) {
            removeIntakeFromRequestedState();
        } else {
            addIntakeToRequestedState();
        }
    }

    public Command toggleIntakeInRequestedStateCommand() {
        return runOnce(this::toggleIntakeInRequestedState).withName("ToggleIntakeInRequestedState");
    }

    public void removeScoringFromRequestedState() {
        if(requestedState == MacroState.SCORING) {
            requestedState = MacroState.IDLE;
        }
        else if(requestedState == MacroState.STEADY_STREAM_SCORING) {
            requestedState = MacroState.INTAKING;
        }
    }

    public void addScoringToRequestedState() {
        if(requestedState == MacroState.IDLE) {
            requestedState = MacroState.SCORING;
        }
        else if(requestedState == MacroState.INTAKING) {
            requestedState = MacroState.STEADY_STREAM_SCORING;
        }
        else if(requestedState == MacroState.SHUTTLING) {
            requestedState = MacroState.SCORING;
        }
        else if(requestedState == MacroState.STEADY_STREAM_SHUTTLING) {
            requestedState = MacroState.STEADY_STREAM_SCORING;
        }
    }

    public void requestScoring() {
        addScoringToRequestedState();
    }

    public Command requestScoringCommand() {
        return runOnce(this::requestScoring).withName("RequestScoring");
    }

    public void removeShuttlingFromRequestedState() {
        if(requestedState == MacroState.SHUTTLING) {
            requestedState = MacroState.IDLE;
        }
        else if(requestedState == MacroState.STEADY_STREAM_SHUTTLING) {
            requestedState = MacroState.INTAKING;
        }
    }

    public void addShuttlingToRequestedState() {
        if(requestedState == MacroState.IDLE) {
            requestedState = MacroState.SHUTTLING;
        }
        else if(requestedState == MacroState.INTAKING) {
            requestedState = MacroState.STEADY_STREAM_SHUTTLING;
        }
        else if(requestedState == MacroState.SCORING) {
            requestedState = MacroState.SHUTTLING;
        }
        else if(requestedState == MacroState.STEADY_STREAM_SCORING) {
            requestedState = MacroState.STEADY_STREAM_SHUTTLING;
        }
    }

    public void requestShuttling() {
        addShuttlingToRequestedState();
    }

    public Command requestShuttlingCommand() {
        return runOnce(this::requestShuttling).withName("RequestShuttling");
    }

    public void turnOffLaunchingStates() {
        removeShuttlingFromRequestedState();
        removeScoringFromRequestedState();
    }

    public Command turnOffLaunchingStatesCommand() {
        return runOnce(this::turnOffLaunchingStates).withName("TurnOffLaunchingStates");
    }

    // ==================== Trigger Factory Methods ====================

    /**
     * Creates a Trigger that is true when the current state matches the given state.
     * @param state The MacroState to check against.
     * @return A Trigger that is true when currentState == state.
     */
    public static Trigger whenCurrentState(MacroState state) {
        return new Trigger(() -> currentState == state);
    }

    /**
     * Creates a Trigger that is true when the desired state matches the given state.
     * @param state The MacroState to check against.
     * @return A Trigger that is true when desiredState == state.
     */
    public static Trigger whenDesiredState(MacroState state) {
        return new Trigger(() -> requestedState == state);
    }

    /**
     * Creates a Trigger that is true when a state has a specific status.
     * @param state The MacroState to check.
     * @param status The Status to check for.
     * @return A Trigger that is true when state.getStatus() == status.
     */
    public static Trigger whenStateHasStatus(MacroState state, Status status) {
        return new Trigger(() -> state.getStatus() == status);
    }

    /**
     * Creates a Trigger that is true when the current state matches and has READY status.
     * Useful for triggering actions only when a state is fully prepared.
     * @param state The MacroState to check.
     * @return A Trigger that is true when currentState == state AND status == READY.
     */
    public static Trigger whenCurrentStateReady(MacroState state) {
        return new Trigger(() -> currentState == state && state.getStatus() == Status.READY);
    }

    /**
     * Creates a Trigger that is true when the current state matches and has WAITING status.
     * Useful for triggering spin-up or preparation actions.
     * @param state The MacroState to check.
     * @return A Trigger that is true when currentState == state AND status == WAITING.
     */
    public static Trigger whenCurrentStateWaiting(MacroState state) {
        return new Trigger(() -> currentState == state && state.getStatus() == Status.WAITING);
    }



    public static Trigger whenIntakeNotCurrent() {
        return new Trigger(
            () -> currentState != MacroState.INTAKING && currentState != MacroState.STEADY_STREAM_SCORING && currentState != MacroState.STEADY_STREAM_SHUTTLING
        );
    }

    public static Trigger whenIntakeNotDesired() {
        return new Trigger(
            () -> requestedState != MacroState.INTAKING && requestedState != MacroState.STEADY_STREAM_SCORING && requestedState != MacroState.STEADY_STREAM_SHUTTLING
        );
    }

    @Override
    public void addPathPlannerCommands() {
        PathPlannerCommands.addCommand("Request Idle State", this.requestStateCommand(MacroState.IDLE));
        PathPlannerCommands.addCommand("Request Scoring State", this.requestStateCommand(MacroState.SCORING));
        PathPlannerCommands.addCommand("Request Shuttling State", this.requestStateCommand(MacroState.SHUTTLING));
        PathPlannerCommands.addCommand("Request Intaking State", this.requestStateCommand(MacroState.INTAKING));
        PathPlannerCommands.addCommand("Request Climbing State", this.requestStateCommand(MacroState.CLIMBING));
        PathPlannerCommands.addCommand("Request ClimbAndScore State", this.requestStateCommand(MacroState.CLIMB_AND_SCORE));
        PathPlannerCommands.addCommand("Request SS Scoring State", this.requestStateCommand(MacroState.STEADY_STREAM_SCORING));
        PathPlannerCommands.addCommand("Request SS Shuttling State", this.requestStateCommand(MacroState.STEADY_STREAM_SHUTTLING));
        
        PathPlannerCommands.addCommand("Force Idle State", this.setCurrentStateCommand(MacroState.IDLE));
        PathPlannerCommands.addCommand("Force Scoring State", this.setCurrentStateCommand(MacroState.SCORING));
        PathPlannerCommands.addCommand("Force Shuttling State", this.setCurrentStateCommand(MacroState.SHUTTLING));
        PathPlannerCommands.addCommand("Force Intaking State", this.setCurrentStateCommand(MacroState.INTAKING));
        PathPlannerCommands.addCommand("Force Climbing State", this.setCurrentStateCommand(MacroState.CLIMBING));
        PathPlannerCommands.addCommand("Force ClimbAndScore State", this.setCurrentStateCommand(MacroState.CLIMB_AND_SCORE));
        PathPlannerCommands.addCommand("Force SS Scoring State", this.setCurrentStateCommand(MacroState.STEADY_STREAM_SCORING));
        PathPlannerCommands.addCommand("Force SS Shuttling State", this.setCurrentStateCommand(MacroState.STEADY_STREAM_SHUTTLING));

        System.out.println("[StateHandler] Added commands to PathPlanner");
    }
}
