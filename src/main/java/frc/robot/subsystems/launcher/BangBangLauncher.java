package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Ports.LauncherPorts.kFixedLauncherMotor;
import static frc.robot.Ports.LauncherPorts.kFixedLauncherMotorFollower;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;
import lombok.Getter;

public class BangBangLauncher extends SubsystemBase{
    private TalonFX mainMotor;
    private TalonFX followingMotor;

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
    private TalonFXConfiguration followerMotorConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)
        );

    @Getter
    boolean atGoal;

    @Getter
    boolean tooFarForTorqueCurrent;

    // Preferences
    private Pref<Double> torqueCurrentOutput = SKPreferences.attach("TorqueCurrent Output (A)", 54.0);
    private Pref<Double> torqueCurrentControlTolerance = SKPreferences.attach("TorqueCurrentControl Tolerance (rps)", 0.85);
    private Pref<Double> torqueCurrentControlDebounce = SKPreferences.attach("TorqueCurrentControl Debounce (sec)", 0.025)
        .onChange(
            (newDebounce) -> {torqueCurrentDebouncer = new Debouncer(newDebounce, DebounceType.kFalling);}
        );
    private Pref<Double> dutyCycleFF = SKPreferences.attach("DutyCycle FF (%/rps)", 0.01489);
    private Pref<Double> atGoalDebounce = SKPreferences.attach("AtGoalVelocity Debounce (sec)", 0.2)
        .onChange((newDebounce) -> {atGoalDebouncer = new Debouncer(newDebounce, DebounceType.kFalling);});

    // Signal Debouncers
    private Debouncer torqueCurrentDebouncer =
        new Debouncer(torqueCurrentControlDebounce.get(), DebounceType.kFalling);
    private Debouncer atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);

    private boolean isTorqueCurrentRunning = false;

    @Getter
    ControlMode activeMode = ControlMode.COAST;
    
    private AngularVelocity targetVelocity = RotationsPerSecond.zero();

    private DutyCycleOut dutyCycleBangBang = new DutyCycleOut(0);
    private TorqueCurrentFOC torqueCurrentBangBang = new TorqueCurrentFOC(0);
    private Follower followerControl = new Follower(kFixedLauncherMotor.ID, MotorAlignmentValue.Opposed);
    private CoastOut coastControl = new CoastOut();

    public BangBangLauncher() {
        mainMotor = new TalonFX(kFixedLauncherMotor.ID, CANBus.roboRIO());
        followingMotor = new TalonFX(kFixedLauncherMotorFollower.ID, CANBus.roboRIO());

        var mainMotorStatus = mainMotor.getConfigurator().apply(mainMotorConfig);
        var followerMotorStatus = followingMotor.getConfigurator().apply(followerMotorConfig);

        if(!mainMotorStatus.isOK()) {
            DriverStation.reportError(
                "Main motor config failed: " + mainMotorStatus, false);
        }
        if(!followerMotorStatus.isOK()) {
            DriverStation.reportError(
                "Follower motor config failed: " + followerMotorStatus, false);
        }

        followingMotor.setControl(followerControl);
    }

    @Override
    public void periodic() {
        switch(activeMode) {
            case DUTY_CYCLE_BANG_BANG:
                mainMotor.setControl(dutyCycleBangBang.withEnableFOC(true).withOutput(
                    MathUtil.clamp(targetVelocity.in(RotationsPerSecond) * dutyCycleFF.get(), -0.75, 0.75)));
                break;
            case TORQUE_CURRENT_BANG_BANG:
                mainMotor.setControl(torqueCurrentBangBang.withOutput(
                    Math.signum(targetVelocity.minus(getVelocity()).in(RotationsPerSecond)) * torqueCurrentOutput.get()));
                break;
            case COAST:
                mainMotor.setControl(coastControl);
                break;
        }
        SmartDashboard.putData("BBLauncher", this);
    }

    /**
     * Runs the flywheel at a given velocity.
     * @param rotationsPerSecond The target velocity in rotations per second.
     */
    private void runVelocity(double rotationsPerSecond) {
        boolean inTolerance =
            Math.abs(getVelocity().in(RotationsPerSecond) - rotationsPerSecond)
                <= torqueCurrentControlTolerance.get();
        boolean tooFar = 
            Math.abs(getVelocity().in(RotationsPerSecond) - rotationsPerSecond)
                > 16 * torqueCurrentControlTolerance.get(); // Ooooooo magic number 16
        boolean runTorqueCurrent = torqueCurrentDebouncer.calculate(!inTolerance && !tooFar);

        atGoal = atGoalDebouncer.calculate(inTolerance);
        tooFarForTorqueCurrent = tooFar;

        isTorqueCurrentRunning = runTorqueCurrent;

        activeMode =
            isTorqueCurrentRunning
                ? ControlMode.TORQUE_CURRENT_BANG_BANG
                : ControlMode.DUTY_CYCLE_BANG_BANG;
        
        targetVelocity = RotationsPerSecond.of(rotationsPerSecond);
    }

    /** Stops the flywheel. */
    private void stop() {
        activeMode = ControlMode.COAST;
        targetVelocity = RotationsPerSecond.zero();
        atGoal = false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("AtGoal", () -> atGoal, null);
        builder.addDoubleProperty(
            "Target Velocity (rps)", 
            () -> targetVelocity.in(RotationsPerSecond), 
            (newTarget) -> runVelocity(newTarget));
        builder.addDoubleProperty("Current Velocity (rps)", () -> getVelocity().in(RotationsPerSecond), null);
        builder.addStringProperty("Control Mode", () -> activeMode.toString(), null);
        builder.addBooleanProperty("Too Far", () -> tooFarForTorqueCurrent, null);
    }

    public AngularVelocity getVelocity() {
        return mainMotor.getVelocity().getValue();
    }

    public enum ControlMode {
        DUTY_CYCLE_BANG_BANG,
        TORQUE_CURRENT_BANG_BANG,
        COAST
    }

    public Command runFixedSpeedCommand(Supplier<AngularVelocity> velocity) {
        return runEnd(() -> runVelocity(velocity.get().in(RotationsPerSecond)), this::stop);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
}
