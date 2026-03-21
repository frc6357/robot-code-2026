package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Konstants.IntakeConstants.IntakePosition;
import static frc.robot.Konstants.FeederConstants.kFeederRunningVoltage;
import static frc.robot.Konstants.IndexerConstants.kIndexerFullVoltage;
import static frc.robot.Konstants.LauncherConstants.DualLauncher.kDefaultTargetRPS;

import frc.robot.subsystems.turret.SK26Turret;
import frc.robot.subsystems.intake.SK26IntakePivot;
import frc.robot.subsystems.intake.SK26IntakeRollers;
import frc.robot.subsystems.indexer.SK26Indexer;
import frc.robot.subsystems.feeder.SK26Feeder;
import frc.robot.subsystems.launcher.mechanisms.BangBangLauncher;
import frc.robot.subsystems.launcher.mechanisms.SK26DualLauncher;
import frc.robot.subsystems.climb.SK26Climb;

/**
 * SmartDashboard-driven test suite for independently testing each subsystem.
 *
 * <h2>Usage</h2>
 * <ol>
 *   <li>Enable the robot in <b>Test</b> mode from Driver Station.</li>
 *   <li>Open SmartDashboard / Shuffleboard.</li>
 *   <li>Each subsystem has a boolean toggle under the {@code TestSuite/} table.</li>
 *   <li>Set a toggle to {@code true} to run that subsystem's test.
 *       Set it back to {@code false} (or just disable the robot) to stop.</li>
 * </ol>
 *
 * <h2>Safety</h2>
 * <ul>
 *   <li><b>Turret</b> will only run when the intake is deployed (GROUND position)
 *       because a stowed intake mechanically interferes with the turret.</li>
 *   <li>Each test command requires its own subsystem, so two tests that share
 *       a subsystem cannot run simultaneously — WPILib's scheduler handles this.</li>
 *   <li>All tests stop the motor / return to idle when the toggle is turned off.</li>
 * </ul>
 */
public class TestSuite {

    private static final String TABLE = "TestSuite";

    /** How long (seconds) each subsystem runs during the "Run All" sequence. */
    private static final double RUN_ALL_STEP_SECONDS = 3.0;

    /**
     * Call once from {@link RobotContainer} after all subsystems are created.
     * Registers SmartDashboard toggles and binds them to test commands.
     */
    public static void init() {
        // Accumulates (name, commandFactory) pairs for the Run-All sequence.
        // We use suppliers (lambdas) so each call to buildRunAll creates fresh
        // command instances — a WPILib command can only be scheduled once.
        List<TestStep> runAllSteps = new ArrayList<>();
        // ---- Intake Pivot ----
        if (RobotContainer.m_intakePivotInstance != null) {
            SK26IntakePivot pivot = RobotContainer.m_intakePivotInstance;

            registerTest("Intake Pivot Deploy", pivot.setIntakePivotTargetCommand(IntakePosition.GROUND)
                .andThen(Commands.idle(pivot))
                .finallyDo(() -> pivot.setPositionerPosition(IntakePosition.ZERO))
                .withName("Test_IntakePivotDeploy"));

            runAllSteps.add(new TestStep("Intake Pivot Deploy", () ->
                pivot.setIntakePivotTargetCommand(IntakePosition.GROUND)
                    .andThen(Commands.idle(pivot))
                    .finallyDo(() -> pivot.setPositionerPosition(IntakePosition.ZERO))));
        }

        // ---- Intake Rollers ----
        if (RobotContainer.m_intakeRollersInstance != null) {
            SK26IntakeRollers rollers = RobotContainer.m_intakeRollersInstance;

            registerTest("Intake Rollers", rollers.runAtVoltageCommand(6.0)
                .withName("Test_IntakeRollers"));

            runAllSteps.add(new TestStep("Intake Rollers", () ->
                rollers.runAtVoltageCommand(6.0)));
        }

        // ---- Indexer ----
        if (RobotContainer.m_indexerInstance != null) {
            SK26Indexer indexer = RobotContainer.m_indexerInstance;

            registerTest("Indexer", indexer.feedCommand(kIndexerFullVoltage)
                .withName("Test_Indexer"));

            runAllSteps.add(new TestStep("Indexer", () ->
                indexer.feedCommand(kIndexerFullVoltage)));
        }

        // ---- Feeder ----
        if (RobotContainer.m_feederInstance != null) {
            SK26Feeder feeder = RobotContainer.m_feederInstance;

            registerTest("Feeder", feeder.feedCommand(kFeederRunningVoltage)
                .withName("Test_Feeder"));

            runAllSteps.add(new TestStep("Feeder", () ->
                feeder.feedCommand(kFeederRunningVoltage)));
        }

        // ---- BangBang Launcher ----
        if (RobotContainer.m_BBlauncherInstance != null) {
            BangBangLauncher bbLauncher = RobotContainer.m_BBlauncherInstance;

            registerTest("BangBang Launcher",
                bbLauncher.runVelocityCommand(() -> edu.wpi.first.units.Units.RotationsPerSecond.of(20))
                    .withName("Test_BBLauncher"));

            runAllSteps.add(new TestStep("BangBang Launcher", () ->
                bbLauncher.runVelocityCommand(() -> edu.wpi.first.units.Units.RotationsPerSecond.of(20))));
        }

        // ---- Dual Launcher ----
        if (RobotContainer.m_dualLauncherInstance != null) {
            SK26DualLauncher dualLauncher = RobotContainer.m_dualLauncherInstance;

            registerTest("Dual Launcher", dualLauncher.runVelocityCommand(kDefaultTargetRPS / 2.0)
                .withName("Test_DualLauncher"));

            runAllSteps.add(new TestStep("Dual Launcher", () ->
                dualLauncher.runVelocityCommand(kDefaultTargetRPS / 2.0)));
        }

        // ---- Turret (requires intake deployed) ----
        if (RobotContainer.m_turretInstance != null) {
            SK26Turret turret = RobotContainer.m_turretInstance;
            SK26IntakePivot pivot = RobotContainer.m_intakePivotInstance;

            // Safety: only allow turret test when intake is deployed
            Command turretTest;
            java.util.function.Supplier<Command> turretFactory;

            if (pivot != null) {
                turretTest = Commands.sequence(
                    pivot.setIntakePivotTargetCommand(IntakePosition.GROUND),
                    Commands.waitUntil(pivot::isPositionerAtTarget).withTimeout(2.0),
                    buildTurretSweep(turret)
                ).finallyDo(() -> {
                    turret.stop();
                    pivot.setPositionerPosition(IntakePosition.ZERO);
                }).withName("Test_Turret");

                turretFactory = () -> Commands.sequence(
                    pivot.setIntakePivotTargetCommand(IntakePosition.GROUND),
                    Commands.waitUntil(pivot::isPositionerAtTarget).withTimeout(2.0),
                    buildTurretSweep(turret)
                ).finallyDo(() -> {
                    turret.stop();
                    pivot.setPositionerPosition(IntakePosition.ZERO);
                });
            } else {
                turretTest = buildTurretSweep(turret)
                    .finallyDo(() -> turret.stop())
                    .withName("Test_Turret_NoIntake");

                turretFactory = () -> buildTurretSweep(turret)
                    .finallyDo(() -> turret.stop());
            }

            registerTest("Turret Sweep", turretTest);
            // Turret gets extra time: intake deploy + 4 sweep positions × 4s each
            runAllSteps.add(new TestStep("Turret Sweep", turretFactory, 18.0));
        }

        // ---- Climb ----
        if (RobotContainer.m_climbInstance != null) {
            SK26Climb climb = RobotContainer.m_climbInstance;

            registerTest("Climb Up", climb.climbUpCommand()
                .withName("Test_ClimbUp"));
            registerTest("Climb Down", climb.climbDownCommand()
                .withName("Test_ClimbDown"));

            runAllSteps.add(new TestStep("Climb Up", () -> climb.climbUpCommand()));
            runAllSteps.add(new TestStep("Climb Down", () -> climb.climbDownCommand()));
        }

        // ==================== Run All ====================
        registerRunAll(runAllSteps);
    }

    // ==================== Helpers ====================

    /**
     * Builds a turret sweep command that exercises the full safe rotation
     * range (±170°) without ever exceeding the limits.
     *
     * <p>Uses only {@code setAngleDegrees()} which clamps to ±170° — the
     * turret will never wrap or exceed its mechanical limits.
     *
     * <p>Sequence (repeating):
     * <ol>
     *   <li>Go to +170° (positive limit)</li>
     *   <li>Go to 0° (center)</li>
     *   <li>Go to -170° (negative limit)</li>
     *   <li>Go to 0° (center)</li>
     * </ol>
     */
    private static Command buildTurretSweep(SK26Turret turret) {
        return Commands.repeatingSequence(
            // Sweep to positive limit
            Commands.runOnce(() -> turret.setAngleDegrees(170.0)),
            Commands.waitUntil(turret::atTarget).withTimeout(4.0),
            // Return to center
            Commands.runOnce(() -> turret.setAngleDegrees(0.0)),
            Commands.waitUntil(turret::atTarget).withTimeout(4.0),
            // Sweep to negative limit
            Commands.runOnce(() -> turret.setAngleDegrees(-170.0)),
            Commands.waitUntil(turret::atTarget).withTimeout(4.0),
            // Return to center
            Commands.runOnce(() -> turret.setAngleDegrees(0.0)),
            Commands.waitUntil(turret::atTarget).withTimeout(4.0)
        );
    }

    /**
     * Creates a SmartDashboard boolean toggle and binds it to a command.
     * When the toggle is set to {@code true}, the command starts.
     * When set back to {@code false}, the command is cancelled (motors stop).
     *
     * @param name    Display name (will appear as "TestSuite/{name}")
     * @param command The test command — should be a continuous (non-finishing)
     *                command that stops its motors in {@code end()}.
     */
    private static void registerTest(String name, Command command) {
        String key = TABLE + "/" + name;

        // Initialize the toggle to false
        SmartDashboard.putBoolean(key, false);

        // Create a trigger from the dashboard boolean
        Trigger toggle = new Trigger(() -> SmartDashboard.getBoolean(key, false));

        // whileTrue: runs the command while the toggle is on, cancels when off
        toggle.whileTrue(command);
    }

    // ==================== Run All ====================

    /**
     * Registers the "Run All Tests" button on SmartDashboard. When toggled on,
     * it sequences through every registered test step, running each for
     * {@link #RUN_ALL_STEP_SECONDS} (or the step's custom duration), logging
     * the current step name, then moving to the next. Toggling off cancels
     * immediately and stops whatever is running.
     */
    private static void registerRunAll(List<TestStep> steps) {
        if (steps.isEmpty()) return;

        String key = TABLE + "/Run All Tests";
        String statusKey = TABLE + "/Current Test";
        SmartDashboard.putBoolean(key, false);
        SmartDashboard.putString(statusKey, "---");

        Trigger toggle = new Trigger(() -> SmartDashboard.getBoolean(key, false));

        // Use defer() so a fresh command sequence is built each time the
        // button is pressed — WPILib commands can only be scheduled once.
        toggle.whileTrue(
            Commands.defer(() -> buildRunAll(steps, statusKey), java.util.Set.of())
                .finallyDo(() -> {
                    SmartDashboard.putString(statusKey, "---");
                    SmartDashboard.putBoolean(key, false);
                    Logger.recordOutput("TestSuite/Running", false);
                })
                .withName("TestSuite_RunAll")
        );
    }

    /**
     * Builds a sequential command that runs each test step for its duration,
     * logging the current step name to SmartDashboard and AdvantageScope.
     */
    private static Command buildRunAll(List<TestStep> steps, String statusKey) {
        List<Command> sequence = new ArrayList<>();

        sequence.add(Commands.runOnce(() -> {
            Logger.recordOutput("TestSuite/Running", true);
            Logger.recordOutput("TestSuite/TotalSteps", steps.size());
        }));

        for (int i = 0; i < steps.size(); i++) {
            TestStep step = steps.get(i);
            final int stepNum = i + 1;

            // Log which step is running
            sequence.add(Commands.runOnce(() -> {
                String label = String.format("[%d/%d] %s", stepNum, steps.size(), step.name);
                SmartDashboard.putString(statusKey, label);
                Logger.recordOutput("TestSuite/CurrentStep", label);
            }));

            // Run the test command for the step's duration, then it auto-cancels
            // (which triggers the command's end() → motors stop)
            sequence.add(step.factory.get()
                .withTimeout(step.durationSec)
                .withName("RunAll_" + step.name));
        }

        // Mark completion
        sequence.add(Commands.runOnce(() -> {
            SmartDashboard.putString(statusKey, "DONE ✓");
            Logger.recordOutput("TestSuite/CurrentStep", "DONE");
        }));

        return Commands.sequence(sequence.toArray(new Command[0]));
    }

    // ==================== TestStep record ====================

    /**
     * Pairs a human-readable name with a command factory (supplier) and a
     * duration for the Run-All sequence.
     */
    private static class TestStep {
        final String name;
        final java.util.function.Supplier<Command> factory;
        final double durationSec;

        TestStep(String name, java.util.function.Supplier<Command> factory) {
            this(name, factory, RUN_ALL_STEP_SECONDS);
        }

        TestStep(String name, java.util.function.Supplier<Command> factory, double durationSec) {
            this.name = name;
            this.factory = factory;
            this.durationSec = durationSec;
        }
    }
}
