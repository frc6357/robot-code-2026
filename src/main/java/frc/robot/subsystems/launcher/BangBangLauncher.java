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

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.preferences.Pref;
import frc.lib.preferences.SKPreferences;

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

    private boolean atGoal;

    Pref<Double> torqueCurrentControlTolerance = SKPreferences.attach("TorqueCurrentControl Tolerance (rps)", 0.85);
    Pref<Double> torqueCurrentControlDebounce = SKPreferences.attach("TorqueCurrentControl Debounce (sec)", 0.025)
        .onChange(
            (newDebounce) -> {torqueCurrentDebouncer = new Debouncer(newDebounce, DebounceType.kFalling);}
        );
    Pref<Double> atGoalDebounce = SKPreferences.attach("AtGoalVelocity Debounce (sec)", 0.2);
    Pref<Double> torqueCurrentOutput = SKPreferences.attach("TorqueCurrent Output (A)", 60.0);
    Pref<Double> dutyCycleOutput = SKPreferences.attach("DutyCycle Output (%)", 0.35);

    private Debouncer torqueCurrentDebouncer =
        new Debouncer(torqueCurrentControlDebounce.get(), DebounceType.kFalling);
    private Debouncer atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);

    private boolean isTorqueCurrentRunning = false;

    private ControlMode activeMode = ControlMode.COAST;
    private AngularVelocity targetVelocity;

    private DutyCycleOut dutyCycleBangBang = new DutyCycleOut(0);
    private TorqueCurrentFOC torqueCurrentBangBang = new TorqueCurrentFOC(0);
    private Follower followerControl = new Follower(kFixedLauncherMotor.ID, MotorAlignmentValue.Opposed);
    private CoastOut coastControl = new CoastOut();

    public BangBangLauncher() {
        mainMotor = new TalonFX(kFixedLauncherMotor.ID, CANBus.roboRIO());
        followingMotor = new TalonFX(kFixedLauncherMotorFollower.ID, CANBus.roboRIO());

        mainMotor.getConfigurator().apply(mainMotorConfig);
        followingMotor.getConfigurator().apply(mainMotorConfig);

        followingMotor.setControl(followerControl);
    }

    @Override
    public void periodic() {
        switch(activeMode) {
            case DUTY_CYCLE_BANG_BANG:
                mainMotor.setControl(dutyCycleBangBang.withEnableFOC(true).withOutput(dutyCycleOutput.get()));
                break;
            case TORQUE_CURRENT_BANG_BANG:
                mainMotor.setControl(torqueCurrentBangBang.withOutput(torqueCurrentOutput.get()));
                break;
            case COAST:
                mainMotor.setControl(coastControl);
                break;
        }
    }

    private void runVelocity(double rotationsPerSecond) {
        boolean inTolerance =
            Math.abs(getVelocity().in(RotationsPerSecond) - rotationsPerSecond)
                <= torqueCurrentControlTolerance.get();
        boolean runTorqueCurrent = torqueCurrentDebouncer.calculate(inTolerance);
        atGoal = atGoalDebouncer.calculate(inTolerance);

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
        builder.addDoubleProperty("Target Velocity (rps)", () -> targetVelocity.in(RotationsPerSecond), null);
        builder.addDoubleProperty("Current Velocity (rps)", () -> getVelocity().in(RotationsPerSecond), null);
        builder.addStringProperty("Control Mode", () -> activeMode.toString(), null);
    }

    private AngularVelocity getVelocity() {
        return mainMotor.getVelocity().getValue();
    }

    private enum ControlMode {
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
