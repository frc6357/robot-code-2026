package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Konstants.DriveConstants.kMaxAngularRate;
import static frc.robot.Konstants.LauncherConstants.kUnJamLauncherRPS;
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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import frc.lib.preferences.SKPreferences;
import frc.lib.preferences.Pref;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
        
        public static final LinearAcceleration kMaxTeleopLinAcceleration = kMaxSpeed.div(Seconds.of(0.33));
        public static final AngularAcceleration kMaxTeleopRotAcceleration = kMaxAngularRate.div(Seconds.of(0.33));

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
            kOperatorControlled(
                new SKTargetPoint(new Translation2d(0, 0), "Operator")
            ),
            kBlueHub(
                new SKTargetPoint(new Translation2d(4.622, 4.0295), "Blue Hub")
            ),
            kRedHub(
                new SKTargetPoint(new Pose2d(11.929, 4.0295, Rotation2d.k180deg), "Red Hub")
            );

            public SKTargetPoint point;

            private TargetPoint(SKTargetPoint point) {
                this.point = point;
            }
        }
    }

    public static final class SwerveConstants
    {
        // Robot Dimension values
    
        // swerve chassis width and length in inches 
        public static final double kChassisLength = 27.5;
        public static final double kChassisWidth = 27.5; 
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
            4.5, 5.1, 
            540, 720, 
            12, false);

        /** Slower, smoother constraints for FuelHunt pathfinding — gentle curves, no snapping. */
        public static final PathConstraints kFuelHuntConstraints = new PathConstraints(
            4.0, 1.2,
            120, 180,
            12, false);
    }

    // ==================== Fuel Hunt Tuning ====================
    // ▼▼▼  CHANGE THESE TO ADJUST FUEL HUNT BEHAVIOUR  ▼▼▼
    //
    // Robot speed / acceleration during fuel hunts are in
    //   AutoConstants.kFuelHuntConstraints  (PathConstraints)
    //   ↑ Increase maxVelocity / maxAcceleration for a faster robot.
    //
    // The constants below control the fuel-hunt decision logic:
    public static final class FuelHuntConstants {

        /** Balls to collect before heading home. (200 on field in sim.) */
        public static final int    kCollectionTarget    = 50;

        /** Max extra metres a detour may add vs. the direct-return cost.
         *  ↑ bigger = robot chases fuel farther off the direct path.
         *  ↓ smaller = tighter hunt corridor. */
        public static final double kMaxDetourExtraM     = 1.5;

        /** End-velocity (m/s) when arriving at a fuel cluster.
         *  Higher = plows through faster (less accurate aim).
         *  Lower  = more precise but slower cycle time. */
        public static final double kFuelGoalEndVel      = 4.0;

        /** End-velocity (m/s) when arriving at the trench entry. */
        public static final double kEntryGoalEndVel     = 2.5;

        /** Safety-timeout seconds — generous; primary exit is the target. */
        public static final double kMaxHuntTimeSec      = 30.0;

        /** Per-leg timeout seconds — prevents a single pathfind from stalling. */
        public static final double kLegTimeoutSec       = 4.0;

        /** Proximity (m) at which a fuel-leg is considered "arrived" and
         *  the robot immediately starts the next leg, no pause. */
        public static final double kFuelProximityM      = 1.0;

        /** Max lateral distance (m) the robot may stray from the entry
         *  trench Y coordinate. Clusters farther away are skipped.
         *  ↑ bigger = robot roams deeper into the neutral zone.
         *  ↓ smaller = robot hugs the trench. */
        public static final double kMaxTrenchLateralM   = 2.5;

        /** Neutral-zone X boundaries — clusters outside are ignored.
         *  ↑ MAX = let robot go deeper;  ↓ MIN = keep robot closer. */
        public static final double kNzMinX              = 4.5;
        public static final double kNzMaxX              = 9.0;

        /** Field bounds for clamping fuel targets (rarely need changing). */
        public static final double kFieldMinX           = kNzMinX;
        public static final double kFieldMaxX           = kNzMaxX;
        public static final double kFieldMinY           = 0.3;
        public static final double kFieldMaxY           = 7.8;
    }
    // ▲▲▲  END FUEL HUNT TUNING  ▲▲▲

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

            public static final boolean kAttached = false;
        }

        /* NOTE: this config should be representative of the Limelight's position when the turret is at 0 degrees */
        public static final class TurretLimelight {
            // Network/pipeline values
            public static final String kName = "limelight-turret"; // NetworkTable name and hostname

            // Translation (in meters) from center of robot
            public static final double kForward = Units.inchesToMeters(-8.0); // (z) meters forward of center; negative is backwards
            public static final double kRight = Units.inchesToMeters(-9.5); // (x) meters right of center; negative is left
            public static final double kUp = Units.inchesToMeters(19.625); // (y) meters up of center; negative is down (how did you get a limelight down there???)

            // Rotation of limelight (in degrees and yaw)
            public static final double kRoll = 180; // (roll) degrees tilted clockwise/ccw from 0° level [think plane wings tilting cw/ccw]
            public static final double kPitch = 1; // (pitch) degrees tilted up/down from 0° level [think plane nose tilting up/down]
            public static final double kYaw = 0; // (yaw) yaw rotated clockwise/ccw from 0° North [think of a compass facing cw/ccw]

            public static final boolean kAttached = false;
        }
        public static final class LimelightThree {
            // Network/pipeline values
            public static final String kName = "limelight-three"; // NetworkTable name and hostname

            // Translation (in meters) from center of robot
            public static final double kForward = Inches.of(-5).in(Meters); // (z) meters forward of center; negative is backwards
            public static final double kRight = Inches.of(6).in(Meters); // (x) meters right of center; negative is left
            public static final double kUp = Inches.of(13.5).in(Meters); // (y) meters up of center; negative is down (how did you get a limelight down there???)

            // Rotation of limelight (in degrees and yaw)
            public static final double kRoll = 0; // (roll) degrees tilted clockwise/ccw from 0° level [think plane wings tilting cw/ccw]
            public static final double kPitch = 0; // (pitch) degrees tilted up/down from 0° level [think plane nose tilting up/down]
            public static final double kYaw = 180; // (yaw) faces backward (same direction as intake)

            public static final boolean kAttached = true;
        }

        public static final class LimelightFour {
            // Network/pipeline values
            public static final String kName = "limelight-four"; // NetworkTable name and hostname

            // Translation (in meters) from center of robot
            public static final double kForward = Inches.of(-9).in(Meters); // (z) meters forward of center; negative is backwards
            public static final double kRight = Inches.of(6).in(Meters); // (x) meters right of center; negative is left
            public static final double kUp = Inches.of(13.5).in(Meters); // (y) meters up of center; negative is down (how did you get a limelight down there???)

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

    public static final class IndexerConstants 
    {
        // Indexer feed speed in Rotations Per Second (RPS)
        public static final double kIndexerFullSpeed = 8.0;

        // Indexer idle speed in Rotations Per Second (RPS)
        public static final double kIndexerIdleSpeed = 0.0;

        // Indexer unjam parameters
        public static final double kIndexerUnjamReverseRPS = -4.0;
        public static final double kIndexerUnjamReverseDuration = 0.25;

        public static final double kIndexerUnjamWaitDuration = 0.25;

        public static final double kIndexerUnjamForwardRPS = 5.0;
        public static final double kIndexerUnjamForwardDuration = 0.25;

        public static final Distance kIndexerHeight = Inches.of(18);

        // Max voltage output for indexer motor (for brownout protection)
        public static final double kMaxIndexerVoltage = 10.0;
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
    
    public static final class SystemConstants
    {
        // Brownout detection
        // public static final double kBrownoutVoltageThreshold = 7.0; // Voltage threshold for brownout detection
        // public static final int kPdhCAN_ID = 63;
        // public static final PowerDistribution PDH = new PowerDistribution(kPdhCAN_ID, ModuleType.kRev);
        // public static final Trigger kBrownoutTrigger = new Trigger(() -> PDH.getVoltage() < kBrownoutVoltageThreshold);

    }

    public static final class LightsConstants
    {
        public static final int kNumLedOnBot = 60;
        public static final double kLightsOffBrightness = 0.0;
        public static final double kLightsOnBrightness = 0.5;

        public static final int kLightsPWMHeader = 9; // PWM Header on the RoboRIO that the lights are connected to (stupid value for now - change later)])
        public static final int kLEDBufferLength = 60; // Number of LEDs on the robot (stupid value for now - change later)

        public static final Color kSKCream = new Color(233 / 255.0, 235 / 255.0, 229 / 255.0);
        public static final Color kSKTeal = new Color(104 / 255.0, 185 / 255.0, 196 / 255.0);
        public static final Color kSKBlue = new Color(81 / 255.0, 171 / 255.0, 185 / 255.0);
        public static final Color kSKDarkBlue = new Color(0 / 255.0, 118 / 255.0, 133 / 255.0);

        public static final Time kDefaultStrobeSeconds = Seconds.of(0.1);

        // Wave animation constants - control the speed and appearance of the wave effect
        public static final double kWaveSpeedCyclesPerSecond = 0.35; // How fast the wave travels along the LED strip (cycles per second)
        public static final double kWaveSpatialCycles = 2.0; // How many complete wave patterns fit across the entire LED strip
        public static final double kWaveColorCycleSec = 2.2; // How long it takes for the color gradient to cycle through all colors (seconds)

    }   

    public static final class TurretConstants
    {
        public static enum TurretPosition
        {
            /** Set the turret angle to 90 degrees **/
            kTurretLeftPosition(90.0),
            /** Set the turret angle to 0 degrees **/
            kTurretZeroPosition(0.0);

            public final double angle;
            TurretPosition(double angle)
            {
                this.angle = angle;
            }
        }

        // Turret position limits and tolerances
        public static final double kTurretMinPosition = -170.0;
        public static final double kTurretMaxPosition = 170.0;
        public static final double kTurretAngleTolerance = 0.5;

        // CANcoder / Absolute Encoder constants
        public static final double kTurretEncoderOffset = -0.3828125; // Rotations (-0.5 to +0.5) //-0.111
        public static final boolean kTurretEncoderInverted = false; // Set true if encoder reads backwards
        public static final double kEncoderGearRatio = 2.0; // 2 encoder rotations = 1 turret rotation

        public static final double kTurretMotorGearRatio = 9.444; // 9.444:1 gearing from motor to turret

        // Motor direction - set true if motor spins opposite to encoder direction
        public static final boolean kTurretMotorInverted = true;

        // Turret PID (WPILib PIDController - input is degrees, output is duty cycle)
        public static final double kTurretP = 0.07; //0.00375
        public static final double kTurretI = 0.02;
        public static final double kTurretD = 0.005; //0.00005
        public static final double kMaxTurretOutput = 2.25; // Max duty cycle (0-1) for safety

        public static final AngularVelocity kMaxTurretMMVelocity = DegreesPerSecond.of(440);
        public static final AngularAcceleration kMaxTurretMMAcceleration = DegreesPerSecondPerSecond.of(1320);

        // Turret extra constants
        public static final double kManualTurretSpeed = 360.0; // Degrees per second at full joystick deflection
        public static final double kTurretJoystickDeadband = 0.15;

        // Translation from center of robot to center of turret bearing
        public static final Translation3d kTurretCenter = new Translation3d(Inches.of(-0.125), Inches.of(-8.625), Inches.of(17.5));
    }

    public static final class ClimbConstants
    {
        public static final double kClimbMotorSpeed = .05;
        public static final Double kClimbP = 0.5;
        public static final double kClimbI = 0;
        public static final double kClimbD = 0;
        public static final double kClimbV = 0;
        public static final double kClimbTolerance = 1; //figure out tolerance
        public static final double kCLimbMax = 0; //figure out value of encoder when climb is at max height.
        public static final double kTOne = 70;
        public static final double kClimbReturn = 25;
    }


    public static final class LauncherConstants {

        //initialize PID values
        public static final double kLauncherA = 0.0;
        public static final double kLauncherV = 0.093;
        public static final double kLauncherS = 0.25;

        public static final double kWheelRadius = .0508; //TEMPORARY
        public static final double kShooterTolerance = 0.5; // +/- rps
        public static final double kTargetlaunchVelocity = 5; //meters per second
        public static final double kTargetMotorRPS = 15.665; //matches with kTargetLaunchVelocity
        public static final double kCoastLauncherRPS = 0.25; //RPS of launcher when waiting to shoots
        public static final double kStopLauncher = 0; // velocity/motorRPS of stopped motor
        public static final double kUnJamLauncherRunTime = 0.25; //Time between rotating and stopping the motor during unjamming
        public static final double kUnJamLauncherPauseTime = 0.25; //Time between stopping and rotating the motor during unjamming
        public static final double kUnJamLauncherRPS = 1/kUnJamLauncherRunTime; //Velocity of motor when unjamming

        public static final class Slot0 {
            public static final double kP = 1.8;
            public static final double kI = 0;
            public static final double kD = 0;
        }
        public static final class Slot1 {
            public static final double kP = 0.5;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        // 3D Transform (placeholder - measure from CAD)
        public static final Transform3d kRobotToShooter =
            new Transform3d(
                new Translation3d(0.0, 0.0, 0.5),  // Placeholder: 0.5m height
                new Rotation3d()                    // No rotation offset
            );

        // Phase delay compensation
        public static final double kPhaseDelaySeconds = 0.03;  // MA's tested value

        // Launch angle mode
        public enum LaunchAngleMode { FIXED, ADJUSTABLE }
        public static final LaunchAngleMode kAngleMode = LaunchAngleMode.FIXED;
        public static final Angle kFixedLaunchAngle = Degrees.of(55);

        // Motion compensation (for InterpolatedShotStrategy)
        public static final int kMaxIterations = 20;
        public static final Distance kConvergenceThresholdMeters = Meters.of(0.01);

        // Filtering
        public static final boolean kEnableFiltering = true;
        public static final double kFilterTimePeriodSeconds = 0.1;

        // Strategy selection
        public static final String kDefaultStrategy = "Interpolated";  // "Interpolated" or "Ballistic"

        // Continuous-feed velocity compensation
        public static final boolean kVelocityCompensationEnabled = true;
        public static final double kMinVelocityRatio = 0.85;  // Reject shots below 85% target speed

        // Valid range
        public static final Distance kMinRangeMeters = Meters.of(1.0);
        public static final Distance kMaxRangeMeters = Meters.of(10.0);

        // "Stationary" speed threshold for deciding when to apply motion compensation
        public static final LinearVelocity kStationaryThresholdMetersPerSecond = MetersPerSecond.of(0.3);

        // Placeholder interpolation data (replace with characterization data)
        // These maps would typically be loaded from CSV or built from characterization
        public static InterpolatingDoubleTreeMap createFlywheelSpeedMap() {
            InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();

            // Example data (distance in meters -> flywheel speed in RPM)
            map.put(1.0, 2500.0);
            map.put(2.0, 3000.0);
            map.put(3.0, 3500.0);
            map.put(4.0, 4000.0);
            map.put(5.0, 4500.0);

            return map;
        }

        public static InterpolatingDoubleTreeMap createTimeOfFlightMap() {
            InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();

            // Example data (distance in meters -> time of flight in seconds)
            map.put(1.0, 0.2);
            map.put(2.0, 0.4);
            map.put(3.0, 0.6);
            map.put(4.0, 0.8);
            map.put(5.0, 1.0);

            return map;
        }
    }

    public static final class ExampleConstants
    {
        public static final double kExampleSpeed = 0.5;  //percentage based where 1.0 is max power and 0.0 is minimum
    }

    public static final class IntakeConstants
    {
        public static enum IntakePosition
        {
            /** Set the intake angle to X degrees **/
            kIntakeGroundPosition(90.0), //TODO This angle needs to be set to a safe angle above the ground
            /** Set the turret angle to 0 degrees (zero position) **/
            kIntakeZeroPosition(0.0); //TODO Make sure to set the ofset in Phoenix Tuner for this :)

            public final double angle;
            IntakePosition(double angle)
            {
                this.angle = angle;
            }
        }

        public static final double kIntakeMotorSpeed = 0.5;
        public static final double kPositionerMotorSpeed = 0.5;

        public static final double kPositionerMotorMinPosition = 0.5;
        public static final double kPositionMotorMaxPosition = 0.5;

        public static final double kMaxIntakeVoltage = 10.0;

        public static final double kIntakeFullSpeed = 8.0;
        public static final double kIntakeIdleSpeed = 2.0;
    }

    public static final String kCANivoreName = "SwerveCANivore";

    /** The file that is used for system instantiation at runtime */
    public static final String SUBSYSTEMFILE = "Subsystems.json";
}

