package frc.robot.subsystems;

import static frc.robot.Ports.LauncherPorts.kAgitatorMotor;
import static frc.robot.Ports.LauncherPorts.kShooterMotor;
import static frc.robot.Ports.LauncherPorts.kTurretMotor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SK26Turret extends SubsystemBase 
{
    //TODO: Moved motor variable declaration outside of constructor since it will be needed in other methods
    //TODO: Changed motor type to TalonFX since we're using krakens

    // Motors
    private TalonFX turretMotor;
    private TalonFX shooterMotor;
    private TalonFX agitatorMotor;

    // Some physical constants (turret related)
    private final MotionMagicDutyCycle motionMagic = new MotionMagicDutyCycle(0);
    private static final double kMotorRotPerTurretRot = 12.8; //Ideally motor rotations per turret rotation
    private static final double kDegreesPerMotorRotation = 360.0 / kMotorRotPerTurretRot; 
    private static final double kMinAngleDegrees = -170.0; //TODO Replace my generic angle values with true value
    private static final double kMaxAngleDegrees = 170.0;
    private static final double kCruiseVelocity = 60; //rotations/sec
    private static final double kAcceleration = 30000; //rotations/sec^2
    private static final double kExtraDegrees = 40.0; //Degrees beyond 180 degrees that the turret can rotate without "snapping its own neck"

    // More physical constants (shooter related)
    private final VelocityDutyCycle shooterVelocityControl = new VelocityDutyCycle(0);
    private static final double kWheelRadiusMeters = 0.0; //TODO Change these when it's given to us
    private static final double kShooterEfficiency = 0.0; //Friction factor
    private static final double kGearRatioShooter = 0.0;

    // A few shooter variables I kinda want for later
    private boolean isShooting = false;
    private double exitVelocity = 0.0;
    private static final double shooterSpeedTolerance = 0.5; //Arbitrary tolerance, but it would be rotations/sec

    // PID values
    private static final double kTurretP = 0.0;
    private static final double kTurretI = 0.0;
    private static final double kTurretD = 0.0;
    private static final double kTurretF = 0.0;

    private static final double kShooterP = 0.0;
    private static final double kShooterI = 0.0;
    private static final double kShooterD = 0.0;
    private static final double kShooterF = 0.0;

    private static final double kFeederP = 0.0;
    private static final double kFeederI = 0.0;
    private static final double kFeederD = 0.0;
    private static final double kFeederF = 0.0;

    public SK26Turret() 
    {
        turretMotor = new TalonFX(kTurretMotor.ID);
        shooterMotor = new TalonFX(kShooterMotor.ID);
        agitatorMotor = new TalonFX(kAgitatorMotor.ID);

        configureTurretMotor();
        configureShooterMotor();
        configureFeederMotor();
        zeroTurret();
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

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degreesToMotorRotations(kMaxAngleDegrees);
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degreesToMotorRotations(kMinAngleDegrees);

        turretMotor.getConfigurator().apply(config);
        turretMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void configureShooterMotor()
    {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kShooterP;
        config.Slot0.kI = kShooterI;
        config.Slot0.kD = kShooterD;
        config.Slot0.kV = kShooterF;

        shooterMotor.getConfigurator().apply(config);
        shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    private void configureFeederMotor() 
    {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kFeederP;
        config.Slot0.kI = kFeederI;
        config.Slot0.kD = kFeederD;
        config.Slot0.kV = kFeederF;

        agitatorMotor.getConfigurator().apply(config);
        agitatorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    /*
     *  Turret-related methods
     */
    public void setAngleDegrees(double angleDegrees)
    {
        angleDegrees = clamp(angleDegrees, kMinAngleDegrees, kMaxAngleDegrees);
        motionMagic.Position = degreesToMotorRotations(angleDegrees);
        turretMotor.setControl(motionMagic);
    }

    // Something to be noted: getting a Rotation2d's angle using inputModulus in degrees returns values from (-180, 180]
    public void setRotation2d(Rotation2d rotation)
    {
        // Get the current angle in degrees
        double currentAngle = getAngleDegrees();

        // Get the target angle in degrees
        double targetAngle = MathUtil.inputModulus(rotation.getDegrees(), -180, 180);

        // If the sign of the current angle and target angle aren't opposites,
        // the turret will not need to cross the 180/-180 boundary
        if(Math.signum(currentAngle) + Math.signum(targetAngle) != 0) {
            // Run the method as usual
            setAngleDegrees(targetAngle);
            return;
        }

        // Check to see how far beyond the 180 degree mark the turret plans to rotate
        double targetAngle180DegDiff = 180 - Math.abs(targetAngle);

        // If it is attempting to rotate more than the extraDegrees beyond 180 degrees, use the default method to rotate the turret
        if(targetAngle180DegDiff > kExtraDegrees) {
            // This effectively snaps the turret around the long way, avoiding "snapping its own neck"
            setAngleDegrees(targetAngle);
            return;
        }

        double newTargetAngle;
        // Create a new, temporary target angle that is cocentric with the original target angle
        if(Math.abs(currentAngle) < 180) {
            // Find the degrees difference in order for the turret to cross beyond 180 
            // degrees and also reach the target angle 
            double totalAngleDifference = (180 - Math.abs(currentAngle)) + targetAngle180DegDiff;

            newTargetAngle = currentAngle + (totalAngleDifference * Math.signum(currentAngle));
        }
        // If the current angle has already passed beyond 180 degrees
        else {
            newTargetAngle = Math.signum(currentAngle) * (180 + Math.abs(targetAngle180DegDiff));
        }
        setAngleDegrees(newTargetAngle);
    }

    public double getAngleDegrees()
    {
        return motorRotationsToDegrees(turretMotor.getPosition().getValueAsDouble());
    }
    private static double degreesToMotorRotations(double degrees)
    {
        return degrees / kDegreesPerMotorRotation;
    }
    private static double motorRotationsToDegrees(double rotations)
    {
        return rotations * kDegreesPerMotorRotation;
    }
    private static double clamp(double val, double min, double max)
    {
        return Math.max(min, Math.min(max, val));
    }
    public void zeroTurret()
    {
        turretMotor.setPosition(0);
    }
    public void manualRotate(double dutyCycle)
    {
        turretMotor.set(dutyCycle);
    }

    /*
     *  Shooter-related methods
     */
    public void shootWithExitVelocity(double exitVelocityMetersPerSecond) 
    {
        // Hopefully calculates a wheel's surface velocity, which accounts for friction
        double wheelSurfaceVelocity = exitVelocityMetersPerSecond / kShooterEfficiency;

        // Converts the above velocity to motor rotations
        double wheelAngularVelocityRadPerSec = wheelSurfaceVelocity / kWheelRadiusMeters; // rad/s
        double motorRotationsPerSecond = wheelAngularVelocityRadPerSec / (2 * Math.PI) * kGearRatioShooter;

        shooterVelocityControl.Velocity = motorRotationsPerSecond;
        shooterMotor.setControl(shooterVelocityControl);
    }
    public void startShooting(double exitVelocityMetersPerSecond)
    {
        exitVelocity = exitVelocityMetersPerSecond;
        isShooting = true;
        shootWithExitVelocity(exitVelocityMetersPerSecond);
    }
    public boolean isShooterAtSpeed()
    {
        double motorRPS = shooterMotor.getVelocity().getValueAsDouble();
        double targetMotorRPS = (exitVelocity / kShooterEfficiency) / kWheelRadiusMeters / (2 * Math.PI) * kGearRatioShooter;

        return Math.abs(motorRPS - targetMotorRPS) < shooterSpeedTolerance;
    }
    public void feedFuel()
    {
        if (isShooting && isShooterAtSpeed())
        {
            agitatorMotor.set(0.7); //Arbitrary percent
        }
        else
        {
            agitatorMotor.set(0.0);
        }
    }
    public void stopShooting()
    {
        isShooting = false;
        shooterMotor.set(0.0);
        agitatorMotor.set(0.0);
    }

    @Override
    public void periodic() {}
}
