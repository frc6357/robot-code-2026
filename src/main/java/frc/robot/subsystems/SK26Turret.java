package frc.robot.subsystems;

// Turret constant imports (from Konstants file)
import static frc.robot.Konstants.TurretConstants.kTurretP;
import static frc.robot.Konstants.TurretConstants.kTurretI;
import static frc.robot.Konstants.TurretConstants.kTurretD;
import static frc.robot.Konstants.TurretConstants.kTurretMinPosition;
import static frc.robot.Konstants.TurretConstants.kTurretMaxPosition;
import static frc.robot.Konstants.TurretConstants.kTurretAngleTolerance;
import static frc.robot.Konstants.TurretConstants.kGearRatio;

// Turret ports imports (from Ports file)
import static frc.robot.Ports.LauncherPorts.kTurretMotor;

// Preferences (from Pref & SKPreferences files)
import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;

// Phoenix-related imports
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

// WPI-related imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SK26Turret extends SubsystemBase 
{
    // Launcher motor delcaration
    TalonFX turretMotor;
    TalonFXConfiguration motorConfig;
    MotorOutputConfigs outputConfigs;

    // Launcher PID controller declarations
    PhoenixPIDController turretPID;
    Slot0Configs turretPID0;

    // Angle tracking declarations
    double motorTargetPosition;
    double motorCurrentPosition;
    double target;
    boolean atTarget;

    // PID Preferences
    Pref<Double> turretkPPref = SKPreferences.attach("turretP", 1.0)
        .onChange((newValue) -> turretPID.setP(newValue));
    Pref<Double> turretkIPref = SKPreferences.attach("turretI", 0.0)
        .onChange((newValue) -> turretPID.setI(newValue));
    Pref<Double> turretkDPref = SKPreferences.attach("turretD", 0.1)
        .onChange((newValue) -> turretPID.setD(newValue));

    public SK26Turret() 
    {
        // Motor initialization
        turretMotor = new TalonFX(kTurretMotor.ID);
        motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // PID Controller initialization
        turretPID = new PhoenixPIDController(kTurretP, kTurretI, kTurretD);
        turretPID.reset();
        turretPID.setTolerance(kTurretAngleTolerance);

        turretPID0 = new Slot0Configs()
            .withKP(turretkPPref.get())
            .withKI(turretkIPref.get())
            .withKD(turretkDPref.get());

        motorConfig.withSlot0(turretPID0);

        turretMotor.setPosition(0.0);
    }

    public void runTurret(double turretSpeed) 
    {
        if((getMotorPosition() > (kTurretMaxPosition - kTurretAngleTolerance)) && Math.signum(turretSpeed) > 0) 
        {
            stop();
            return;
        }
        else
        { 
            // Sets a velocity to target via pid and supplies an average duty cycle in volts
            turretMotor.setControl(new PositionDutyCycle(turretSpeed));
        }
    }

    public double getMotorSpeed() 
    {
        return turretMotor.getVelocity().getValueAsDouble(); // Rotations / sec
    }

    public double getMotorPosition() 
    {
        double motorRotations = turretMotor.getPosition().getValueAsDouble(); // Rotations
        double angle = motorRotations / (kGearRatio * 360.0); // Degrees conversion
        return angle;
    }

    public double getTargetPosition() 
    {
        double targetMotorRotations = turretMotor.getClosedLoopReference().getValueAsDouble(); // Rotations
        double angle = targetMotorRotations / (kGearRatio * 360.0); // Degrees conversion
        return angle;
    }

    public void stop() 
    {
        turretMotor.stopMotor();
    }

    public boolean isAtTargetPosition() 
    {
        return Math.abs(getTargetPosition() - getMotorPosition()) <= kTurretAngleTolerance;
    }

    @Override
    public void periodic() 
    {
        motorCurrentPosition = getMotorPosition(); 

        SmartDashboard.putNumber("Turret Velocity", getMotorSpeed());
        SmartDashboard.putNumber("Current Turret Position", getMotorPosition());
        SmartDashboard.putNumber("Target Turret Position", getTargetPosition());
        SmartDashboard.putBoolean("At Target Position", isAtTargetPosition());
    }
}