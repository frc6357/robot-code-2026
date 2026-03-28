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
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;
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
public class SK26DualLauncher extends SubsystemBase {

    // Motors
    private final TalonFX bottomMotor;
    private final TalonFX topMotor;

    // Control requests (reused each cycle to avoid allocation)
    private final VelocityVoltage bottomVelocityControl = new VelocityVoltage(0).withEnableFOC(true);
    private final VelocityVoltage topVelocityControl    = new VelocityVoltage(0).withEnableFOC(true);
    private final CoastOut coastControl = new CoastOut();

    // State
    private double topTargetVelocity = 0.0;
    private double bottomTargetVelocity = 0.0;
    private double cachedBottomVelocityRPS = 0.0;
    private double cachedTopVelocityRPS = 0.0;

    @Getter
    private boolean running = false;

    @Getter
    private LauncherTuning launcherTuning = new LauncherTuning("DualLauncher");

    private Pref<Double> flywheelTargetSpeed = SKPreferences.attach("DualLauncher/ManualTargetSpeed (rps)", 35.0);
    private Pref<Double> topRollerRatioToBottom = SKPreferences.attach("DualLauncher/Top:BottomRollerRatio", 1.1);

    // ==================== Live PID/FF Tuning (Phoenix Tuner X style) ====================
    // Change any value on SmartDashboard/Shuffleboard and the gain is hot-applied
    // to the TalonFX immediately — no redeploy required.
    //
    // Bottom Roller gains
    private final Pref<Double> bottomKS = SKPreferences.attach("DualLauncher/Bottom/kS", DualLauncher.BottomRoller.kS)
        .onChange(v -> applyBottomSlot0());
    private final Pref<Double> bottomKV = SKPreferences.attach("DualLauncher/Bottom/kV", DualLauncher.BottomRoller.kV)
        .onChange(v -> applyBottomSlot0());
    private final Pref<Double> bottomKA = SKPreferences.attach("DualLauncher/Bottom/kA", DualLauncher.BottomRoller.kA)
        .onChange(v -> applyBottomSlot0());
    private final Pref<Double> bottomKP = SKPreferences.attach("DualLauncher/Bottom/kP", DualLauncher.BottomRoller.kP)
        .onChange(v -> applyBottomSlot0());
    private final Pref<Double> bottomKI = SKPreferences.attach("DualLauncher/Bottom/kI", DualLauncher.BottomRoller.kI)
        .onChange(v -> applyBottomSlot0());
    private final Pref<Double> bottomKD = SKPreferences.attach("DualLauncher/Bottom/kD", DualLauncher.BottomRoller.kD)
        .onChange(v -> applyBottomSlot0());

    // Top Roller gains
    private final Pref<Double> topKS = SKPreferences.attach("DualLauncher/Top/kS", DualLauncher.TopRoller.kS)
        .onChange(v -> applyTopSlot0());
    private final Pref<Double> topKV = SKPreferences.attach("DualLauncher/Top/kV", DualLauncher.TopRoller.kV)
        .onChange(v -> applyTopSlot0());
    private final Pref<Double> topKA = SKPreferences.attach("DualLauncher/Top/kA", DualLauncher.TopRoller.kA)
        .onChange(v -> applyTopSlot0());
    private final Pref<Double> topKP = SKPreferences.attach("DualLauncher/Top/kP", DualLauncher.TopRoller.kP)
        .onChange(v -> applyTopSlot0());
    private final Pref<Double> topKI = SKPreferences.attach("DualLauncher/Top/kI", DualLauncher.TopRoller.kI)
        .onChange(v -> applyTopSlot0());
    private final Pref<Double> topKD = SKPreferences.attach("DualLauncher/Top/kD", DualLauncher.TopRoller.kD)
        .onChange(v -> applyTopSlot0());

    // Test setpoint — change on the dashboard, then hold the tuning command button
    private final Pref<Double> tuningSetpointRPS = SKPreferences.attach(
        "DualLauncher/Tuning/Setpoint (rps)", DualLauncher.kDefaultTargetRPS);

    // Velocity tolerance (live-tunable)
    private final Pref<Double> velocityToleranceRPS = SKPreferences.attach(
        "DualLauncher/Tuning/Tolerance (rps)", DualLauncher.kVelocityToleranceRPS);

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
                    .withInverted(InvertedValue.CounterClockwise_Positive)
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

    // ==================== Live Tuning: Hot-Apply Methods ====================

    /**
     * Builds a Slot0Configs from the current bottom roller Pref values
     * and hot-applies it to the bottom motor. Called automatically whenever
     * any bottom roller gain is changed on the dashboard.
     */
    private void applyBottomSlot0() {
        Slot0Configs slot0 = new Slot0Configs()
            .withKS(bottomKS.get())
            .withKV(bottomKV.get())
            .withKA(bottomKA.get())
            .withKP(bottomKP.get())
            .withKI(bottomKI.get())
            .withKD(bottomKD.get());

        var status = bottomMotor.getConfigurator().apply(slot0);
        if (!status.isOK()) {
            DriverStation.reportError(
                "[DualLauncher] Bottom Slot0 hot-apply failed: " + status, false);
        }
        Logger.recordOutput("DualLauncher/Tuning/BottomSlot0Applied", true);
    }

    /**
     * Builds a Slot0Configs from the current top roller Pref values
     * and hot-applies it to the top motor. Called automatically whenever
     * any top roller gain is changed on the dashboard.
     */
    private void applyTopSlot0() {
        Slot0Configs slot0 = new Slot0Configs()
            .withKS(topKS.get())
            .withKV(topKV.get())
            .withKA(topKA.get())
            .withKP(topKP.get())
            .withKI(topKI.get())
            .withKD(topKD.get());

        var status = topMotor.getConfigurator().apply(slot0);
        if (!status.isOK()) {
            DriverStation.reportError(
                "[DualLauncher] Top Slot0 hot-apply failed: " + status, false);
        }
        Logger.recordOutput("DualLauncher/Tuning/TopSlot0Applied", true);
    }

    @Override
    public void periodic() {
        // Cache velocity reads (one CAN read per motor per cycle)
        cachedBottomVelocityRPS = bottomMotor.getVelocity().getValueAsDouble();
        cachedTopVelocityRPS    = topMotor.getVelocity().getValueAsDouble();

        if (running) {
            bottomMotor.setControl(bottomVelocityControl.withVelocity(bottomTargetVelocity));
            topMotor.setControl(topVelocityControl.withVelocity(topTargetVelocity));
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
        bottomTargetVelocity = rps;
        topTargetVelocity = bottomTargetVelocity * topRollerRatioToBottom.get();
        running = true;
    }

    /** Stop both rollers and coast to a halt. */
    public void stop() {
        topTargetVelocity = 0.0;
        running = false;
    }

    /**
     * @return true if both rollers are within tolerance of the target velocity.
     */
    public boolean atTargetVelocity() {
        if (!running) return false;
        double tolerance = velocityToleranceRPS.get();
        return Math.abs(cachedBottomVelocityRPS - bottomTargetVelocity) <= tolerance
            && Math.abs(cachedTopVelocityRPS - topTargetVelocity)    <= tolerance;
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
    public double getTopTargetVelocity() {
        return topTargetVelocity;
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

    public Command runVelocityFromPrefCommand() {
        return runEnd(
            () -> runVelocity(flywheelTargetSpeed.get()), 
            this::stop);
    }

    /** Returns a command that immediately stops both rollers. */
    public Command stopCommand() {
        return runOnce(this::stop);
    }

    /**
     * Returns a command that runs both rollers at the dashboard tuning setpoint
     * while held, and stops when released. Reads the setpoint each cycle so you
     * can change it live without re-scheduling.
     * 
     * <p>Bind this to a button for pit/practice tuning — adjust gains and
     * setpoint on the dashboard, hold the button, watch the response in
     * AdvantageScope, iterate.
     */
    public Command tuningCommand() {
        return runEnd(
            () -> runVelocity(tuningSetpointRPS.get()),
            this::stop
        ).withName("DualLauncherTuning");
    }

    // ==================== Telemetry ====================

    private void telemeterize() {
        Logger.recordOutput("DualLauncher/Running", running);
        Logger.recordOutput("DualLauncher/Target Velocity (rps)", bottomTargetVelocity);
        Logger.recordOutput("DualLauncher/Bottom Velocity (rps)", cachedBottomVelocityRPS);
        Logger.recordOutput("DualLauncher/Top Velocity (rps)", cachedTopVelocityRPS);
        Logger.recordOutput("DualLauncher/Top Velocity (RPM)", cachedTopVelocityRPS * 60.0);
        Logger.recordOutput("DualLauncher/Bottom Velocity (RPM)", cachedBottomVelocityRPS * 60.0);
        Logger.recordOutput("DualLauncher/Bottom Error (rps)", bottomTargetVelocity - cachedBottomVelocityRPS);
        Logger.recordOutput("DualLauncher/Top Error (rps)", topTargetVelocity - cachedTopVelocityRPS);
        Logger.recordOutput("DualLauncher/At Target", atTargetVelocity());

        // Log active gains so they appear alongside the response in AdvantageScope
        Logger.recordOutput("DualLauncher/Tuning/Bottom/kS", bottomKS.get());
        Logger.recordOutput("DualLauncher/Tuning/Bottom/kV", bottomKV.get());
        Logger.recordOutput("DualLauncher/Tuning/Bottom/kA", bottomKA.get());
        Logger.recordOutput("DualLauncher/Tuning/Bottom/kP", bottomKP.get());
        Logger.recordOutput("DualLauncher/Tuning/Bottom/kI", bottomKI.get());
        Logger.recordOutput("DualLauncher/Tuning/Bottom/kD", bottomKD.get());
        Logger.recordOutput("DualLauncher/Tuning/Top/kS", topKS.get());
        Logger.recordOutput("DualLauncher/Tuning/Top/kV", topKV.get());
        Logger.recordOutput("DualLauncher/Tuning/Top/kA", topKA.get());
        Logger.recordOutput("DualLauncher/Tuning/Top/kP", topKP.get());
        Logger.recordOutput("DualLauncher/Tuning/Top/kI", topKI.get());
        Logger.recordOutput("DualLauncher/Tuning/Top/kD", topKD.get());
        Logger.recordOutput("DualLauncher/Tuning/Setpoint (rps)", tuningSetpointRPS.get());
        Logger.recordOutput("DualLauncher/Tuning/Tolerance (rps)", velocityToleranceRPS.get());
    }
}
