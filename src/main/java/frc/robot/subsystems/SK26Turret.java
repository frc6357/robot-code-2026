package frc.robot.subsystems;

// Imported constants from the Ports file
import static frc.robot.Ports.LauncherPorts.kTurretMotor;
import static frc.robot.Ports.LauncherPorts.kTurretEncoder;

// Imported constants from the Konstants file
import static frc.robot.Konstants.TurretConstants.kAcceleration;
import static frc.robot.Konstants.TurretConstants.kCruiseVelocity;
import static frc.robot.Konstants.TurretConstants.kDegreesPerMotorRotation;
import static frc.robot.Konstants.TurretConstants.kExtraDegrees;
import static frc.robot.Konstants.TurretConstants.kMaxAngleDegrees;
import static frc.robot.Konstants.TurretConstants.kMinAngleDegrees;
import static frc.robot.Konstants.TurretConstants.kTurretZeroPosition;

// Phoenix/Kraken related imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

// WPI related imports
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SK26Turret extends SubsystemBase 
{
    // Motors
    private TalonFX turretMotor;

    // Absolute Encoder
    private final CANcoder turretEncoder = new CANcoder(kTurretEncoder.ID);

    // Some physical constants (turret related)
    private final MotionMagicDutyCycle motionMagic = new MotionMagicDutyCycle(0);
    public static double lastTargetAngle = 0.0;

    // Simulation Constants (Temporary)
    private static final double kTurretGearRatio = 3.0; // 3:1 gear ratio
    private static final double kTurretMOI = 0.002;

    // PID values
    private static final double kTurretP = 1.0;
    private static final double kTurretI = 0.0;
    private static final double kTurretD = 0.1;
    private static final double kTurretF = 0.0;

    public SK26Turret() 
    {
        turretMotor = new TalonFX(kTurretMotor.ID);

        configureTurretMotor();
        configureTurretEncoder();
        zeroTurret();
    }

    private void configureTurretEncoder()
    {
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        // Set this encoder as counterclockwise positive
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        // Magnet offset in rotations (0 → 1 rotation)
        encoderConfig.MagnetSensor.MagnetOffset = kTurretZeroPosition / 360.0;

        // Apply config
        turretEncoder.getConfigurator().apply(encoderConfig);

        // Tell TalonFX to use this encoder as its primary feedback
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = 
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        // Apply directly to motor
        turretMotor.getConfigurator().apply(config);
    }

    private void configureTurretMotor() 
    {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.Slot0.kP = kTurretP;
        config.Slot0.kI = kTurretI;
        config.Slot0.kD = kTurretD;
        config.Slot0.kV = kTurretF;

        config.MotionMagic.MotionMagicCruiseVelocity = kCruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = kAcceleration;

        config.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RemoteCANcoder;

        config.Feedback.SensorToMechanismRatio = kTurretGearRatio;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            kMaxAngleDegrees / 360.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            kMinAngleDegrees / 360.0;

        turretMotor.getConfigurator().apply(config);
        turretMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    /*
     *  Turret-related methods
     */
    public void setAngleDegrees(double angleDegrees)
    {
        angleDegrees = clamp(angleDegrees, kMinAngleDegrees, kMaxAngleDegrees);
        lastTargetAngle = angleDegrees;
        motionMagic.Position = angleDegrees / 360.0; // mechanism rotations
        turretMotor.setControl(motionMagic);
    }

    // Something to be noted: getting a Rotation2d's angle using inputModulus in degrees returns values from (-180, 180]
    public void setRotation2d(Rotation2d rotation)
    {
        // Get the current angle in degrees
        double currentAngle = getAngleDegrees();
        // Get the target angle in degrees
        double targetAngle = MathUtil.inputModulus(rotation.getDegrees(), -180, 180);
        // Check to see how far beyond the 180 degree mark the turret plans to rotate
        double targetAngle180DegDiff = 180 - Math.abs(targetAngle);
        double newTargetAngle;

        // If the sign of the current angle and target angle aren't opposites,
        // the turret will not need to cross the 180/-180 boundary
        if(Math.signum(currentAngle) + Math.signum(targetAngle) != 0) {
            // Run the method as usual
            setAngleDegrees(targetAngle);
            return;
        }

        // If it is attempting to rotate more than the extraDegrees beyond 180 degrees, use the default method to rotate the turret
        if(targetAngle180DegDiff > kExtraDegrees) 
        {
            // This effectively snaps the turret around the long way, avoiding "snapping its own neck"
            setAngleDegrees(targetAngle);
            return;
        }
        // Create a new, temporary target angle that is cocentric with the original target angle
        if(Math.abs(currentAngle) < 180) 
        {
            // Find the degrees difference in order for the turret to cross beyond 180 
            // degrees and also reach the target angle 
            double totalAngleDifference = (180 - Math.abs(currentAngle)) + targetAngle180DegDiff;
            newTargetAngle = currentAngle + (totalAngleDifference * Math.signum(currentAngle));
            setAngleDegrees(newTargetAngle);
        }

        // If the current angle has already passed beyond 180 degrees
        else 
        {
            newTargetAngle = Math.signum(currentAngle) * (180 + Math.abs(targetAngle180DegDiff));
            setAngleDegrees(newTargetAngle);
        }
    }

    // Holds the turret at its last target position.
    public void holdPosition() 
    {
        setAngleDegrees(lastTargetAngle);
    }

    // Returns the current turret angle in degrees.
    public double getAngleDegrees()
    {
        return turretMotor.getPosition().getValueAsDouble() * 360.0;
    }

    // Conversion methods.
    // private static double degreesToMotorRotations(double degrees)
    // {
    //     return degrees / kDegreesPerMotorRotation;
    // }
    // private static double motorRotationsToDegrees(double rotations)
    // {
    //     return rotations * kDegreesPerMotorRotation;
    // }

    // Clamps a value between a min and max.
    private static double clamp(double val, double min, double max)
    {
        return Math.max(min, Math.min(max, val));
    }

    // Sets the turret to its zero position.
    public void zeroTurret(){}
    
    // Manually rotates the turret at a given duty cycle.
    public void manualRotate(double dutyCycle)
    {
        if (Math.abs(dutyCycle) > 0.01) 
        {
            turretMotor.set(dutyCycle);
            lastTargetAngle = getAngleDegrees();
        }
        else
        {
            holdPosition();
        }
    }

    // DC motor simulation for realistic turret stuff in SimGUI
    private final DCMotorSim turretMotorSim =
    new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1),
            kTurretMOI,
            kTurretGearRatio
        ),
        DCMotor.getKrakenX60Foc(1)
    );

    // Update simulation stuff periodically.. this better work smh
    @Override
    public void simulationPeriodic()
    {
        var simState = turretMotor.getSimState();

        // Supply voltage from battery
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Motor voltage commanded by TalonFX
        var motorVoltage = simState.getMotorVoltageMeasure();

        // Run physics model
        turretMotorSim.setInputVoltage(motorVoltage.in(Units.Volts));
        turretMotorSim.update(0.020);

        // Feed rotor values back into TalonFX
        simState.setRawRotorPosition(
            turretMotorSim.getAngularPosition().times(kTurretGearRatio)
        );
        simState.setRotorVelocity(
            turretMotorSim.getAngularVelocity().times(kTurretGearRatio)
        );
    }

    @Override
    public void periodic() 
    {
        double currentAngle = getAngleDegrees();
        double newTargetAngle = lastTargetAngle;

        SmartDashboard.putData("Turret", this);
        SmartDashboard.putNumber("Current Estimated Position", currentAngle);
        SmartDashboard.putNumber("Current Target Position", newTargetAngle);
    }
}
