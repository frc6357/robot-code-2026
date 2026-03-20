package frc.robot.subsystems.launcher.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Ports.LauncherPorts.kLauncherFrontRollers;
import static frc.robot.Ports.LauncherPorts.kLauncherBackRollers;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;
import frc.lib.subsystems.PathplannerSubsystem;
import frc.robot.subsystems.launcher.moveandshoot.LauncherTuning;
import lombok.Getter;

public class BangBangLauncher extends SubsystemBase implements PathplannerSubsystem {
    private TalonFX mainMotor;
    private TalonFX followingMotor;

    @Getter
    private LauncherTuning launcherTuning = new LauncherTuning("BBLauncher");

    private TalonFXConfiguration mainMotorConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)
        )
        .withFeedback(
            new FeedbackConfigs()
                .withVelocityFilterTimeConstant(0.01)
        )
        .withTorqueCurrent(
            new TorqueCurrentConfigs()
                .withTorqueNeutralDeadband(Amps.of(4))
        );

    @Getter
    private boolean atGoal;

    @Getter
    private boolean tooFarForTorqueCurrent;

    // Preferences
    private Pref<Double> torqueCurrentOutput = SKPreferences.attach("BBLauncher/Prefs/TorqueCurrent Output (A)", 54.0);
    private Pref<Double> torqueCurrentControlTolerance = SKPreferences.attach("BBLauncher/Prefs/TorqueCurrentControl Tolerance (rps)", 0.85);
    private Pref<Double> torqueCurrentControlDebounce = SKPreferences.attach("BBLauncher/Prefs/TorqueCurrentControl Debounce (sec)", 0.025);
    private Pref<Double> voltageFF = SKPreferences.attach("BBLauncher/Prefs/Voltage FF (V/rps)", 0.01489 * 12.0);
    private Pref<Double> atGoalDebounce = SKPreferences.attach("BBLauncher/Prefs/AtGoalVelocity Debounce (sec)", 0.2);
        
    @SuppressWarnings("unused")
    private Pref<Double> targetSpeed = SKPreferences.attach("BBLauncher/Prefs/Target Speed (RPM)", 5000.0)
        .onChange((newSpeed) -> {this.runVelocity(newSpeed / 60.0);});

    // Signal Debouncers
    private final Debouncer torqueCurrentDebouncer =
        new Debouncer(torqueCurrentControlDebounce.get(), DebounceType.kFalling);
    private final Debouncer atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);

    // Wire up debounce time changes after debouncers are initialized
    {
        torqueCurrentControlDebounce.onChange(
            (newDebounce) -> torqueCurrentDebouncer.setDebounceTime(newDebounce));
        atGoalDebounce.onChange(
            (newDebounce) -> atGoalDebouncer.setDebounceTime(newDebounce));
    }

    private boolean isTorqueCurrentRunning = false;

    /** Factor beyond tolerance at which torque-current control is too far behind to be effective. */
    private static final double kTorqueCurrentFarMultiplier = 16.0;

    @Getter
    private ControlMode activeMode = ControlMode.COAST;
    
    /** Target flywheel velocity in rotations per second. */
    private double targetVelocityRps = 0.0;

    /** Cached current motor velocity (rps), refreshed once per periodic cycle. */
    private double cachedVelocityRps = 0.0;

    private VoltageOut voltageBangBang = new VoltageOut(0);
    private TorqueCurrentFOC torqueCurrentBangBang = new TorqueCurrentFOC(0);
    private Follower followerControl = new Follower(kLauncherFrontRollers.ID, MotorAlignmentValue.Opposed);
    private CoastOut coastControl = new CoastOut();

    public BangBangLauncher() {
        mainMotor = new TalonFX(kLauncherFrontRollers.ID, CANBus.roboRIO());
        followingMotor = new TalonFX(kLauncherBackRollers.ID, CANBus.roboRIO());

        var mainMotorStatus = mainMotor.getConfigurator().apply(mainMotorConfig);

        if(!mainMotorStatus.isOK()) {
            DriverStation.reportError(
                "[BangBangLauncher] Main motor config failed: " + mainMotorStatus, false);
        }

        followingMotor.setControl(followerControl);
    }

    @Override
    public void periodic() {
        SKPreferences.refreshIfNeeded();
        cachedVelocityRps = mainMotor.getVelocity().getValueAsDouble();

        switch(activeMode) {
            case VOLTAGE_BANG_BANG:
                mainMotor.setControl(voltageBangBang.withEnableFOC(true).withOutput(
                    MathUtil.clamp(targetVelocityRps * voltageFF.get(), -9.0, 9.0)));
                break;
            case TORQUE_CURRENT_BANG_BANG:
                mainMotor.setControl(torqueCurrentBangBang.withOutput(
                    Math.signum(targetVelocityRps - cachedVelocityRps) * torqueCurrentOutput.get()));
                break;
            case COAST:
                mainMotor.setControl(coastControl);
                break;
        }

        telemeterize();
    }

    /**
     * Runs the flywheel at a given velocity.
     * @param rotationsPerSecond The target velocity in rotations per second.
     */
    private void runVelocity(double rotationsPerSecond) {
        double error = Math.abs(cachedVelocityRps - rotationsPerSecond);
        boolean inTolerance = error <= torqueCurrentControlTolerance.get();
        boolean tooFar = error > kTorqueCurrentFarMultiplier * torqueCurrentControlTolerance.get();

        atGoal = atGoalDebouncer.calculate(inTolerance);
        tooFarForTorqueCurrent = tooFar;

        isTorqueCurrentRunning = torqueCurrentDebouncer.calculate(!inTolerance && !tooFar);
        activeMode =
            isTorqueCurrentRunning
                ? ControlMode.TORQUE_CURRENT_BANG_BANG
                : ControlMode.VOLTAGE_BANG_BANG;
        
        targetVelocityRps = rotationsPerSecond;
    }

    /** Stops the flywheel and resets all control state. */
    private void stop() {
        activeMode = ControlMode.COAST;
        targetVelocityRps = 0.0;
        atGoal = false;
        tooFarForTorqueCurrent = false;
        isTorqueCurrentRunning = false;

        // Feed resting values into debouncers so stale history doesn't
        // produce a wrong output on the first cycle of the next spinup.
        atGoalDebouncer.calculate(false);
        torqueCurrentDebouncer.calculate(false);
    }

    public void telemeterize() {
        Logger.recordOutput("BBLauncher/AtGoal", atGoal);
        Logger.recordOutput("BBLauncher/Target Velocity (rps)", targetVelocityRps);
        Logger.recordOutput("BBLauncher/Current Velocity (rps)", cachedVelocityRps);
        Logger.recordOutput("BBLauncher/Velocity Error (rps)", targetVelocityRps - cachedVelocityRps);
        Logger.recordOutput("BBLauncher/Control Mode", activeMode.toString());
        Logger.recordOutput("BBLauncher/Too Far", tooFarForTorqueCurrent);
    }

    /** Returns the current flywheel velocity in rotations per second. */
    public double getVelocityRps() {
        return cachedVelocityRps;
    }

    public enum ControlMode {
        VOLTAGE_BANG_BANG,
        TORQUE_CURRENT_BANG_BANG,
        COAST
    }

    public Command runVelocityCommand(Supplier<AngularVelocity> velocity) {
        return runEnd(() -> runVelocity(velocity.get().in(RotationsPerSecond)), this::stop);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    @Override
    public void addPathPlannerCommands() {
        throw new UnsupportedOperationException("Unimplemented method 'addPathPlannerCommands'");
    }
}
