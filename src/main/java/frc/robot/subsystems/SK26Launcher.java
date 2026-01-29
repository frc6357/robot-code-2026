package frc.robot.subsystems;

// Imported constants from the Konstants file
import static frc.robot.Ports.LauncherPorts.kLauncherMotor;
import static frc.robot.Konstants.LauncherConstants.kLauncherStopSpeed;
import static frc.robot.Konstants.LauncherConstants.kWheelRadiusMeters;
import static frc.robot.Konstants.LauncherConstants.kGearRatioShooter;
import static frc.robot.Konstants.LauncherConstants.kShooterEfficiency;
import static frc.robot.Konstants.LauncherConstants.shooterSpeedTolerance;

// Phoenix/Kraken related imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

// WPI related imports
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SK26Launcher extends SubsystemBase 
{
    // Motor
    private TalonFX launcherMotor;

    // Duty cycle for PID
    private final VelocityDutyCycle launcherVelocityControl = new VelocityDutyCycle(0);
    
    // A few shooter variables I kinda want for later
    private boolean isShooting = false;
    private double exitVelocity = 0.0;

    // PID values
    private static final double kShooterP = 0.0;
    private static final double kShooterI = 0.0;
    private static final double kShooterD = 0.0;
    private static final double kShooterF = 0.0;

    public SK26Launcher() 
    {
        launcherMotor = new TalonFX(kLauncherMotor.ID);
        configureLauncherMotor();
    }

    public void configureLauncherMotor()
    {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kShooterP;
        config.Slot0.kI = kShooterI;
        config.Slot0.kD = kShooterD;
        config.Slot0.kV = kShooterF;

        launcherMotor.getConfigurator().apply(config);
        launcherMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    /*
     *  Launcher-related methods
     */
    public void launchWithExitVelocity(double exitVelocityMetersPerSecond) 
    {
        // Hopefully calculates a wheel's surface velocity, which accounts for friction
        double wheelSurfaceVelocity = exitVelocityMetersPerSecond / kShooterEfficiency;

        // Converts the above velocity to motor rotations
        double wheelAngularVelocityRadPerSec = wheelSurfaceVelocity / kWheelRadiusMeters; // rad/s
        double motorRotationsPerSecond = wheelAngularVelocityRadPerSec / (2 * Math.PI) * kGearRatioShooter;

        launcherVelocityControl.Velocity = motorRotationsPerSecond;
        launcherMotor.setControl(launcherVelocityControl);
    }
    public void startLaunching(double exitVelocityMetersPerSecond)
    {
        exitVelocity = exitVelocityMetersPerSecond;
        isShooting = true;
        launchWithExitVelocity(exitVelocityMetersPerSecond);
    }
    public boolean isLauncherAtSpeed()
    {
        double motorRPS = launcherMotor.getVelocity().getValueAsDouble();
        double targetMotorRPS = (exitVelocity / kShooterEfficiency) / (kWheelRadiusMeters * (2 * Math.PI) * kGearRatioShooter);

        return Math.abs(motorRPS - targetMotorRPS) < shooterSpeedTolerance;
    }
    public void stopShooting()
    {
        isShooting = false;
        launcherMotor.set(kLauncherStopSpeed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("Launcher", this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("IsShooting", () -> isShooting, null);
    }
}
