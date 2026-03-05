package frc.robot.subsystems.feeder;

import static frc.robot.Konstants.FeederConstants.kMaxFeederVoltage;
import static frc.robot.Konstants.FeederConstants.kFeederIdleVelocity;
import static frc.robot.Ports.LauncherPorts.kFeederMotor;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class SK26Feeder extends SubsystemBase
{
    private final SparkFlex feederMotor;
    private final RelativeEncoder feederEncoder;

    private double targetVoltage = 0.0;

    public SK26Feeder() 
    {
        // ========== Motor Configuration ==========
        feederMotor = new SparkFlex(kFeederMotor.ID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .voltageCompensation(12.0); // Enable voltage compensation for consistent behavior
        feederMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // ========== Encoder Configuration ==========
        feederEncoder = feederMotor.getEncoder();
    }

    /**
     * Sets the voltage of the feeder motor directly.
     * Clamped for safety to prevent brownouts.
     * @param voltage The desired voltage (-12 to +12 volts).
     */
    public void setFeederVoltage(double voltage) {
        // Clamp output for safety
        voltage = MathUtil.clamp(voltage, -kMaxFeederVoltage, kMaxFeederVoltage);
        targetVoltage = voltage;
        feederMotor.setVoltage(voltage);
    }

    /**
     * Sets the velocity of the feeder motor in RPS (Rotations Per Second).
     * Internally converts to voltage control.
     * @param velocity The desired velocity in RPS.
     */
    public void setFeederVelocity(double velocity) {
        // Neo Vortex free speed is ~113 RPS (6784 RPM) at 12V
        // Convert RPS to voltage (approximate open-loop)
        double voltage = (velocity / 113.0) * 12.0;
        setFeederVoltage(voltage);
    }

    public Command setFeederVelCommand(double velocityRPS) {
        return run(() -> setFeederVelocity(velocityRPS));
    }

    public Command setFeederVoltageCommand(double voltage) {
        return run(() -> setFeederVoltage(voltage));
    }

    public void idleFeeder() 
    {
        setFeederVelocity(kFeederIdleVelocity);
    }

    /**
     * Feeds fuel by setting the feeder to the feed speed.
     * @param feederFeedRPS The feed speed in RPS.
     */
    public void feedFuel(double feederFeedRPS) 
    {
        setFeederVelocity(feederFeedRPS);
    }

    @Override
    public void periodic() {}
}
