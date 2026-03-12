package frc.robot.subsystems.launcher.mechanisms;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Konstants.LauncherConstants.DualLauncher;
import static frc.robot.Ports.LauncherPorts.kLauncherFrontRollers;
import static frc.robot.Ports.LauncherPorts.kLauncherBackRollers;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.PathplannerSubsystem;
import frc.robot.subsystems.launcher.moveandshoot.LauncherTuning;
import lombok.Getter;

/**
 * Dual-wheel launcher subsystem with independently PID-controlled top and bottom rollers.
 * 
 * <p>The bottom roller drives the large flywheel (Clockwise Positive) and the top roller
 * drives the smaller contact wheels (CounterClockwise Positive). Both share the same
 * velocity setpoint since the pulley system produces equal tangential speeds at equal
 * angular speeds.
 * 
 * <p>Each motor runs its own VelocityVoltage closed-loop for tighter tracking and
 * independent health monitoring.
 */
public class SK26DualLauncher extends SubsystemBase implements PathplannerSubsystem {

    // Motors
    private final TalonFX bottomMotor;
    private final TalonFX topMotor;

    // Control requests (reused each cycle to avoid allocation)
    private final VelocityVoltage bottomVelocityControl = new VelocityVoltage(0).withEnableFOC(true);
    private final VelocityVoltage topVelocityControl    = new VelocityVoltage(0).withEnableFOC(true);
    private final CoastOut coastControl = new CoastOut();

    // State
    private double targetVelocityRPS = 0.0;
    private double cachedBottomVelocityRPS = 0.0;
    private double cachedTopVelocityRPS = 0.0;

    @Getter
    private boolean running = false;

    @Getter
    private LauncherTuning launcherTuning = new LauncherTuning("DualLauncher");

    public SK26DualLauncher() {
        bottomMotor = new TalonFX(kLauncherFrontRollers.ID, CANBus.roboRIO());
        topMotor    = new TalonFX(kLauncherBackRollers.ID, CANBus.roboRIO());

        configureMotors();
    }

    private void configureMotors() {
        // ========== Bottom roller config (large flywheel) ==========
        TalonFXConfiguration bottomConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withFeedback(
                new FeedbackConfigs()
                    .withVelocityFilterTimeConstant(0.01))
            .withSlot0(
                new Slot0Configs()
                    .withKS(DualLauncher.BottomRoller.kS)
                    .withKV(DualLauncher.BottomRoller.kV)
                    .withKA(DualLauncher.BottomRoller.kA)
                    .withKP(DualLauncher.BottomRoller.kP)
                    .withKI(DualLauncher.BottomRoller.kI)
                    .withKD(DualLauncher.BottomRoller.kD))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(DualLauncher.kSupplyCurrentLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(DualLauncher.kStatorCurrentLimit));

        // ========== Top roller config (smaller contact wheels) ==========
        TalonFXConfiguration topConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withFeedback(
                new FeedbackConfigs()
                    .withVelocityFilterTimeConstant(0.01))
            .withSlot0(
                new Slot0Configs()
                    .withKS(DualLauncher.TopRoller.kS)
                    .withKV(DualLauncher.TopRoller.kV)
                    .withKA(DualLauncher.TopRoller.kA)
                    .withKP(DualLauncher.TopRoller.kP)
                    .withKI(DualLauncher.TopRoller.kI)
                    .withKD(DualLauncher.TopRoller.kD))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(DualLauncher.kSupplyCurrentLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(DualLauncher.kStatorCurrentLimit));

        // Apply configs and report errors
        var bottomStatus = bottomMotor.getConfigurator().apply(bottomConfig);
        var topStatus    = topMotor.getConfigurator().apply(topConfig);

        if (!bottomStatus.isOK()) {
            DriverStation.reportError(
                "[DualLauncher] Bottom motor config failed: " + bottomStatus, false);
        }
        if (!topStatus.isOK()) {
            DriverStation.reportError(
                "[DualLauncher] Top motor config failed: " + topStatus, false);
        }
    }

    @Override
    public void periodic() {
        // Cache velocity reads (one CAN read per motor per cycle)
        cachedBottomVelocityRPS = bottomMotor.getVelocity().getValueAsDouble();
        cachedTopVelocityRPS    = topMotor.getVelocity().getValueAsDouble();

        if (running) {
            bottomMotor.setControl(bottomVelocityControl.withVelocity(targetVelocityRPS));
            topMotor.setControl(topVelocityControl.withVelocity(targetVelocityRPS));
        } else {
            bottomMotor.setControl(coastControl);
            topMotor.setControl(coastControl);
        }

        telemeterize();
    }

    // ==================== Public API ====================

    /**
     * Set the target velocity for both rollers.
     * @param rps Target velocity in rotations per second.
     */
    public void runVelocity(double rps) {
        targetVelocityRPS = rps;
        running = true;
    }

    /** Stop both rollers and coast to a halt. */
    public void stop() {
        targetVelocityRPS = 0.0;
        running = false;
    }

    /**
     * @return true if both rollers are within tolerance of the target velocity.
     */
    public boolean atTargetVelocity() {
        if (!running) return false;
        return Math.abs(cachedBottomVelocityRPS - targetVelocityRPS) <= DualLauncher.kVelocityToleranceRPS
            && Math.abs(cachedTopVelocityRPS - targetVelocityRPS)    <= DualLauncher.kVelocityToleranceRPS;
    }

    /** @return Current bottom roller velocity in RPS. */
    public double getBottomVelocityRPS() {
        return cachedBottomVelocityRPS;
    }

    /** @return Current top roller velocity in RPS. */
    public double getTopVelocityRPS() {
        return cachedTopVelocityRPS;
    }

    /** @return The current target velocity in RPS. */
    public double getTargetVelocityRPS() {
        return targetVelocityRPS;
    }

    // ==================== Commands ====================

    /**
     * Returns a command that runs both rollers at the supplied velocity while held,
     * and stops them when released.
     * @param velocity Supplier of the target angular velocity.
     */
    public Command runVelocityCommand(Supplier<AngularVelocity> velocity) {
        return runEnd(
            () -> runVelocity(velocity.get().in(RotationsPerSecond)),
            this::stop);
    }

    /**
     * Returns a command that runs both rollers at a fixed velocity while held,
     * and stops them when released.
     * @param rps The target velocity in rotations per second.
     */
    public Command runVelocityCommand(double rps) {
        return runEnd(
            () -> runVelocity(rps),
            this::stop);
    }

    /** Returns a command that immediately stops both rollers. */
    public Command stopCommand() {
        return runOnce(this::stop);
    }

    // ==================== Telemetry ====================

    private void telemeterize() {
        Logger.recordOutput("DualLauncher/Running", running);
        Logger.recordOutput("DualLauncher/Target Velocity (rps)", targetVelocityRPS);
        Logger.recordOutput("DualLauncher/Bottom Velocity (rps)", cachedBottomVelocityRPS);
        Logger.recordOutput("DualLauncher/Top Velocity (rps)", cachedTopVelocityRPS);
        Logger.recordOutput("DualLauncher/Bottom Error (rps)", targetVelocityRPS - cachedBottomVelocityRPS);
        Logger.recordOutput("DualLauncher/Top Error (rps)", targetVelocityRPS - cachedTopVelocityRPS);
        Logger.recordOutput("DualLauncher/At Target", atTargetVelocity());
    }

    // ==================== PathPlanner ====================

    @Override
    public void addPathPlannerCommands() {
        // TODO: Register named commands for autonomous launcher control
    }
}
