package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Konstants.OIConstants.kJoystickDeadband;
import static frc.robot.Konstants.OIConstants.kSlowModePercent;

import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.subsystems.drive.GeneratedConstants;
import frc.robot.subsystems.drive.SKTargetPoint;

@SuppressWarnings("unused")
public final class Konstants
{
    public static final class DriveConstants {
        public static final LinearVelocity kMaxSpeed = GeneratedConstants.kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
        public static final LinearVelocity kMaxSpeedFAST = kMaxSpeed.times(1.75);
        public static final LinearVelocity kMaxSpeedSLOW = kMaxSpeed.times(0.3);

        public static final AngularVelocity kMaxAngularRate = RotationsPerSecond.of(1.25); // 3/4 of a rotation per second max angular velocity
        public static final AngularVelocity kMaxAngularRateFAST = kMaxAngularRate.times(2); // 1.5 rotations per second max angular velocity
        public static final AngularVelocity kMaxAngularRateSLOW = kMaxAngularRate.times(0.5); // 1/4 of a rotation per second max angular velocity

        //pigeon ID
        public static final int kPigeonID = 5; //5

        public static double getDeadbandedStick(double rawValue) {
            if (Math.abs(rawValue) < kJoystickDeadband) {
                return 0.0;
            } else {
                double unsignedValue = (Math.abs(rawValue) - kJoystickDeadband)
                        / (1.0 - kJoystickDeadband);
                return (rawValue > 0 ? unsignedValue : -unsignedValue);
            }
        }

        public static final class RotationAligningConstants {
            public static final double kP = 0.85;
            public static final double kI = 0.1;
            public static final double kD = 0.03;

            public static final Rotation2d[] kBumpJumpAngles = new Rotation2d[] {
                Rotation2d.fromDegrees(45),
                Rotation2d.fromDegrees(135),
                Rotation2d.fromDegrees(-45),
                Rotation2d.fromDegrees(-135)
            };
        }
    }

    public static final class TargetPointConstants {
        public enum TargetPoint {
            kOperatorControlled,
            kBlueHub,
            kRedHub
        }

        // Order these the same as the TargetPoint enum
        public static SKTargetPoint[] targetPoints = {
            new SKTargetPoint(new Translation2d(0, 0), "Operator"),
            new SKTargetPoint(new Translation2d(4.622, 4.0295), "Blue Hub"),
            new SKTargetPoint(new Pose2d(11.929, 4.0295, Rotation2d.k180deg), "Red Hub")
        };
    }

    public static final class SwerveConstants
    {
        // Robot Dimension values
    
        // swerve chassis width and length in inches 
        public static final int kChassisLength = 28; // TODO: protobot is 28x28, but final bot will be different
        public static final int kChassisWidth = 28; 
    }

    public static final class AutoConstants
    {
        // Time and speed for rollers in auto
        public static final double kIntakeAutoSpeed = 0.7;
        public static final double kExtakeAutoSpeed = -0.7;
        public static final double kIntakeAutoDurationSeconds = 0.3;  //0.5

        // PID Constants
        public static final PIDConstants kTranslationPIDConstants = new PIDConstants(6.4, 0.05, 0);
        public static final PIDConstants kRotationPIDConstants    = new PIDConstants(6, 0.4, 0.0);

        public static final PPHolonomicDriveController pathConfig = new PPHolonomicDriveController(kTranslationPIDConstants, kRotationPIDConstants);

        public static final PathConstraints kDefaultPathfindingConstraints = new PathConstraints(
            3.5, 3.0, 
            540, 720, 
            12, false);
    }

    public static final class SimulationRobotConstants
    {
        public static final double kPixelsPerMeter = 20;
    
        public static final double kElevatorGearing = 25; // 25:1
        public static final double kCarriageMass =
            4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
        public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
        public static final double kMinElevatorHeightMeters = 0.922; // m
        public static final double kMaxElevatorHeightMeters = 1.62; // m
    }

    public static final class VisionConstants { // Each limelight has a greek letter name and an individual class for their own set of constants
        public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        public static final int kAprilTagPipeline = 0; // Default Apriltag pipeline value for all Limelights

        /* Example:
        public static final class RightLimelight {
            // Network/pipeline values
            public static final String kName = "right-limelight"; // NetworkTable name and hostname

            // Translation (in meters) from center of robot
            public static final double kForward = 0.17145; // (z) meters forward of center; negative is backwards
            public static final double kRight = 0.27305; // (x) meters right of center; negative is left
            public static final double kUp = 0.28575; // (y) meters up of center; negative is down (how did you get a limelight down there???)

            // Rotation of limelight (in degrees and yaw)
            public static final double kRoll = 0; // (roll) degrees tilted clockwise/ccw from 0° level [think plane wings tilting cw/ccw]
            public static final double kPitch = 0; // (pitch) degrees tilted up/down from 0° level [think plane nose tilting up/down]
            public static final double kYaw = 5; // (yaw) yaw rotated clockwise/ccw from 0° North [think of a compass facing cw/ccw]

            public static final boolean kAttached = true;
        }
        */

        public static final class FrontLimelight {
            // Network/pipeline values
            public static final String kName = "limelight-front"; // NetworkTable name and hostname

            // Translation (in meters) from center of robot
            public static final double kForward = 0.3; // (z) meters forward of center; negative is backwards
            public static final double kRight = 0.0; // (x) meters right of center; negative is left
            public static final double kUp = 0.25; // (y) meters up of center; negative is down (how did you get a limelight down there???)

            // Rotation of limelight (in degrees and yaw)
            public static final double kRoll = 0; // (roll) degrees tilted clockwise/ccw from 0° level [think plane wings tilting cw/ccw]
            public static final double kPitch = 0; // (pitch) degrees tilted up/down from 0° level [think plane nose tilting up/down]
            public static final double kYaw = 180; // (yaw) yaw rotated clockwise/ccw from 0° North [think of a compass facing cw/ccw]

            public static final boolean kAttached = true;
        }

        public static final class AlignmentConstants {
            public static double kRejectDistance = 1.4; // 1.4m
        }
    }

    /** Constants that are used when defining filters for controllers */
    public static final class OIConstants
    {
        // Controller constraints
        public static final double kDriveCoeff       = 1;
        public static final double kRotationCoeff    = 1;
        public static final double kJoystickDeadband = 0.15;
        public static final double kSlowModePercent  = 0.3;
        public static final double kSlowModeRotationPercent = 0.5;
        
        public static final double kAccelLimit = 2;

        /** The maximum elevator height in motor rotations, in which driving the robot at maximum 
         * acceleration will not cause the robot to tip over.*/
        public static final double kMaxFullSpeedElevatorHeight = 2.0;
    }

    public static final class LightConstants
    {
        public static final int numLedOnBot = 240;
        public static final double kLightsOffBrightness = 0.0;
        public static final double kLightsOnBrightness = 0.5;
    }

    public static final class IndexerConstants
    {
        public static final double kFeederSpeed = 0.7;
        public static final double kFeederStop = 0.0;
    }

    public static final class LauncherConstants
    {
        // Launcher Speeds
        public static final double kLauncherSpeed = 0.1;
        public static final double kLauncherStopSpeed = 0.0;
        public static final double shooterSpeedTolerance = 0.5; //Arbitrary tolerance, but it would be rotations/sec

        // Physical Constants
        public static final double kWheelRadiusMeters = 0.0; //TODO Change these when it's given to us
        public static final double kShooterEfficiency = 0.0; //Friction factor
        public static final double kGearRatioShooter = 0.0;

    }

    public static final class TurretConstants
    {
        // Turret position limits and tolerances
        public static final double kTurretMinPosition = -180;
        public static final double kTurretMaxPosition = 180;
        public static final double kTurretAngleTolerance = 0.5;

        // CANcoder / Absolute Encoder constants
        public static final double kTurretEncoderOffset = -0.111; // Rotations (-0.5 to +0.5)
        public static final boolean kTurretEncoderInverted = false; // Set true if encoder reads backwards
        public static final double kEncoderGearRatio = 2.0; // 2 encoder rotations per 1 turret rotation

        // Motor direction - set true if motor spins opposite to expected
        public static final boolean kTurretMotorInverted = true;

        // Turret PID (WPILib PIDController - input in degrees, output is duty cycle)
        public static final double kTurretP = 0.02; // Start conservative - increase as needed
        public static final double kTurretI = 0.0;
        public static final double kTurretD = 0.001;
        public static final double kMaxTurretOutput = 0.5; // Max duty cycle (0-1)

        // Turret extra constants
        public static final double kManualTurretSpeed = 90.0; // Degrees per second at full joystick deflection
        public static final double kTurretDeadband = 0.05;
    }



    public static final class ExampleConstants
    {
        public static final double kExampleSpeed = 0.5;  //percentage based where 1.0 is max power and 0.0 is minimum
    }
    
    public static final String kCANivoreName = "SwerveCANivore";

    /** The file that is used for system instantiation at runtime */
    public static final String SUBSYSTEMFILE = "Subsystems.json";
}
