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
        public static final int kPigeonID = 30; //30

        public static double getDeadbandedStick(double rawValue) {
            if (Math.abs(rawValue) < kJoystickDeadband) {
                return 0.0;
            } else {
                double unsignedValue = (Math.abs(rawValue) - kJoystickDeadband)
                        / (1.0 - kJoystickDeadband);
                return (rawValue > 0 ? unsignedValue : -unsignedValue);
            }
        }
    }

    public static final class SwerveConstants
    {
        // //Device Settings and Default States
        
        //swerve motor IDs
        public static final int kFrontLeftDriveMotorID = 1; //1
        public static final int kFrontRightDriveMotorID = 2; //2
        public static final int kBackLeftDriveMotorID = 3; //3
        public static final int kBackRightDriveMotorID = 4; //4
        
        public static final int kFrontLeftTurnMotorID = 11; //11
        public static final int kFrontRightTurnMotorID = 12; //12
        public static final int kBackLeftTurnMotorID = 13; //13
        public static final int kBackRightTurnMotorID = 14; //14
        
        //encoder IDs
        public static final int kFrontLeftEncoderID = 21; //21
        public static final int kFrontRightEncoderID = 22; //22
        public static final int kBackLeftEncoderID = 23; //23
        public static final int kBackRightEncoderID = 24; //24
        
        //Robot Dimension values
    
        //swerve chassis width and length in inches 
        public static final int kChassisLength = 27;
        public static final int kChassisWidth = 27;    
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
        public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

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
        public static final double kTurretAngleTolerance = 0.2;

        // Turret physical constants
        public static final double kGearRatio = 12.8; // Motor rotations per turret rotation
        public static final double kDegrees = 360.0;

        // Turret PID
        public static final double kTurretP = 1.0;
        public static final double kTurretI = 0.0;
        public static final double kTurretD = 0.015;

        // Turret extra constants
        public static final double kManualTurretSpeed = 90.0; // Degrees per second at full joystick deflection
        public static final double kTurretDeadband = 0.05;

        // TODO slated for deletion in the future.
        public static final double kMotorRotPerTurretRot = 12.8; //Ideally motor rotations per turret rotation
        public static final double kDegreesPerMotorRotation = 360.0 / kMotorRotPerTurretRot; 
        public static final double kMinAngleDegrees = -180.0; //TODO Replace my generic angle values with true value
        public static final double kMaxAngleDegrees = 180.0;
        public static final double kCruiseVelocity = 60; //rotations/sec
        public static final double kAcceleration = 30000; //rotations/sec^2
        public static final double kExtraDegrees = 40.0; //Degrees beyond 180 degrees that the turret can rotate without "snapping its own neck"
    }



    public static final class ExampleConstants
    {
        public static final double kExampleSpeed = 0.5;  //percentage based where 1.0 is max power and 0.0 is minimum
    }
    
    public static final String kCANivoreName = "SwerveCANivore";

    /** The file that is used for system instantiation at runtime */
    public static final String SUBSYSTEMFILE = "Subsystems.json";
}
