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
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Konstants.DriveConstants.kMaxAngularRate;
import static frc.robot.Konstants.LauncherConstants.kUnJamLauncherRPS;
import static frc.robot.Konstants.OIConstants.kJoystickDeadband;
import static frc.robot.Konstants.OIConstants.kSlowModePercent;
import static frc.robot.Konstants.TurretConstants.kTurretEncoderOffset;

import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StructPublisher;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.GeneratedConstants;
import frc.robot.subsystems.drive.SKTargetPoint;

@SuppressWarnings("unused")
public final class Konstants
{
    //TODO: This is a HUGE todo- make as many constants as practical as possible into TunableNumbers.

    public static final class DriveConstants {
        
        public static final LinearVelocity kMaxSpeed = GeneratedConstants.kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
        public static final LinearVelocity kMaxSpeedFAST = kMaxSpeed.times(1.75);
        public static final LinearVelocity kMaxSpeedSLOW = kMaxSpeed.times(0.35);
        
        public static final AngularVelocity kMaxAngularRate = RotationsPerSecond.of(1.25); // 3/4 of a rotation per second max angular velocity
        public static final AngularVelocity kMaxAngularRateFAST = kMaxAngularRate.times(2); // 1.5 rotations per second max angular velocity
        public static final AngularVelocity kMaxAngularRateSLOW = kMaxAngularRate.times(0.333); // 1/4 of a rotation per second max angular velocity
        
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
            4.0, 3.65, 
            540, 720, 
            12, false);
        
        public static final PathConstraints kTrenchPathfindingConstraints = new PathConstraints(
            4.0, 3.65, 
            360, 540, 
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

        public static final class BackLimelight {
            // Network/pipeline values
            public static final String kName = "limelight-back"; // NetworkTable name and hostname

            // Translation (in meters) from center of robot
            public static final double kForward = Inches.of(-11.5).in(Meters); // (z) meters forward of center; negative is backwards
            public static final double kRight = Inches.of(11.25).in(Meters); // (x) meters right of center; negative is left
            public static final double kUp = Inches.of(8.75).in(Meters); // (y) meters up of center; negative is down (how did you get a limelight down there???)

            // Rotation of limelight (in degrees and yaw)
            public static final double kRoll = -0.1; // (roll) degrees tilted clockwise/ccw from 0° level [think plane wings tilting cw/ccw]
            public static final double kPitch = 33.5; // (pitch) degrees tilted up/down from 0° level [think plane nose tilting up/down]
            public static final double kYaw = -153; // (yaw) yaw rotated clockwise/ccw from 0° North [think of a compass facing cw/ccw]

            public static final boolean kAttached = true;
        }

        /* NOTE: this config should be representative of the Limelight's position when the turret is at 0 degrees */
        public static final class TurretLimelight {
            // Network/pipeline values
            public static final String kName = "limelight-turret"; // NetworkTable name and hostname

            // Translation (in meters) from center of robot
            public static final double kForward = Units.inchesToMeters(-5.729); // (z) meters forward of center; negative is backwards
            public static final double kRight = Units.inchesToMeters(-12.018); // (x) meters right of center; negative is left
            public static final double kUp = Units.inchesToMeters(17.833); // (y) meters up of center; negative is down (how did you get a limelight down there???)

            // Rotation of limelight (in degrees and yaw)
            public static final double kRoll = -0.5; // (roll) degrees tilted clockwise/ccw from 0° level [think plane wings tilting cw/ccw]
            public static final double kPitch = 17.6; // (pitch) degrees tilted up/down from 0° level [think plane nose tilting up/down]
            public static final double kYaw = 90; // (yaw) yaw rotated clockwise/ccw from 0° North [think of a compass facing cw/ccw]

            public static final boolean kAttached = true;
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

            public static final boolean kAttached = false;
        }

        public static final class AlignmentConstants {
            public static double kRejectDistance = 1.4; // 1.4m
        }

        // The turret pivot point in WPILib robot-space (X=forward, Y=left, Z=up) in meters.
        // Derived from the turret LL position at turret 0° and the known 6.718" offset
        // from the LL to the turret center (behind the LL when it faces left/west).
        //   forward: same as LL (-5.729")
        //   left:    LL is at +12.018" left, pivot is 6.718" behind (toward right = less left) → 12.018 - 6.718 = 5.318" left
        //   up:      same as LL (17.833")
        public static final Translation3d kTurretPivotInRobotSpace = new Translation3d(
            Units.inchesToMeters(-5.729),   // forward  (same as turret LL)
            Units.inchesToMeters(5.318),  // left     (WPILib Y=left; LL right was -6.532, so left = +5.318 from center)
            Units.inchesToMeters(17.833)); // up       (same as turret LL)

        // Use this to check the center of the turret bearing with the 3d render of the robot on AdvantageScope
        // static {
        //     StructPublisher<Translation3d> turretCenterPublisher = NetworkTableInstance.getDefault().getStructTopic("TurretCenterRobotSpace", Translation3d.struct).publish();
        //     turretCenterPublisher.accept(kTurretPivotInRobotSpace);
        // }
    }

    public static final class IndexerConstants 
    {
        // Indexer feed speed in Rotations Per Second (RPS)
        public static final double kIndexerFullVoltage = -6.75;

        // Indexer idle speed in Rotations Per Second (RPS)
        public static final double kIndexerIdleVoltage = 0.0;

        // Indexer unjam parameters
        public static final double kIndexerUnjamReverseRPS = -4.0;
        public static final double kIndexerUnjamReverseDuration = 0.25;

        public static final double kIndexerUnjamWaitDuration = 0.25;

        public static final double kIndexerUnjamForwardRPS = 5.0;
        public static final double kIndexerUnjamForwardDuration = 0.25;

        public static final Distance kIndexerHeight = Inches.of(18);

        // Max voltage output for indexer motor (for brownout protection)
        public static final double kMaxIndexerVoltage = 7.0;
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
        public static final double kTurretAngleTolerance = 6.7; // Degrees of tolerance for considering the turret "at position"

        // CANcoder / Absolute Encoder constants
        public static final double kTurretEncoderOffset = 0.200928 ; // Rotations (-0.5 to +0.5) - negated due to encoder inversion
        public static final boolean kTurretEncoderInverted = true; // Set true if encoder reads backwards
        public static final double kEncoderGearRatio = 2.0; // 2 encoder rotations = 1 turret rotation

        public static final double kTurretMotorGearRatio = 9.444; // 9.444:1 gearing from motor to turret

        // Motor direction - set true if motor spins opposite to encoder direction
        public static final boolean kTurretMotorInverted = true;

        // Gain scheduler constants for turret PID control
        public static final double kTurretGainSchedulerDeadbandDegrees = 0.85; // 0.85 // Degrees of error until the turret's gain scheduler turns on

        // Turret PID (Phoenix6 Slot0 — input is rotations, output is voltage)
        // Converted from old WPILib V/deg gains: multiply by 360 for V/rot
        public static final double kTurretP = 50.0; //50.0  // was 0.07 V/deg (20)
        public static final double kTurretI = 0.0;   // was 0.02 V/(deg·s)
        public static final double kTurretD = 0.0; // 2.0   // was 0.005 V/(deg/s)
        public static final double kTurretS = 0.375; //(0.3)   // Static friction feedforward (volts)
        public static final double kTurretV = 1.9; // 1.85    // Velocity feedforward (volts per rotation per second) 1.0
        public static final double kTurretA = 0.35; // 0.25  // Acceleration feedforward (volts per rotation per second squared) 1.542
        public static final double kMaxTurretOutputVolts = 3.75; // Max voltage output to turret motor (for brownout protection)

        public static final AngularVelocity kMaxTurretMMVelocity = RotationsPerSecond.of(2.0);
        public static final AngularAcceleration kMaxTurretMMAcceleration = RotationsPerSecondPerSecond.of(6.0);
        public static final double kMaxTurretMMJerk = kMaxTurretMMAcceleration.in(RotationsPerSecondPerSecond) * 10; // Jerk is typically 10x acceleration

        // Turret extra constants
        public static final double kManualTurretSpeed = 360.0; // Degrees per second at full joystick deflection
        public static final double kTurretJoystickDeadband = 0.15;

        // Turret coordinate system offset
        // Standard robot-relative: 0° = front, +90° = left (CCW positive)
        // Turret coordinates: 0° = left, +90° = front
        // To convert: turretAngle = standardAngle - kTurretCoordinateOffset
        public static final double kTurretCoordinateOffset = 90.0;

        // Turret lead angle compensation (Option 3 framework with Option 2 defaults)
        // Lead formula: leadAngle = yawVelocity × baseLeadTime × scaleFactor
        // scaleFactor = clamp(|yawVelocity| / referenceVelocity, minScale, maxScale)
        // With minScale=1.0 and maxScale=1.0, this behaves like Option 2 (fixed lookahead)
        public static final double kTurretBaseLeadTimeSeconds = 0.040;   // 40ms base lookahead
        public static final double kTurretReferenceYawVelocity = 30.0;   // deg/s for scaling
        public static final double kTurretMinLeadScale = 1.0;            // Start as Option 2
        public static final double kTurretMaxLeadScale = 1.0;            // Start as Option 2
    }

    public static final class ClimbConstants
    {
        public static final double kClimbMotorSpeed = .05;
        public static final Double kClimbP = 0.5; // TODO: Test for proper value. Needs to be lowered, currently running to fast. Thinking around .3-.4 is a good value.
        public static final double kClimbI = 0;
        public static final double kClimbD = 0;
        public static final double kClimbV = 0;
        public static final double kClimbTolerance = 1; //figure out tolerance
        public static final double kCLimbMax = 0; //figure out value of encoder when climb is at max height.
        public static final double kTOne = 92;
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

        // ==================== Dual Launcher Constants ====================
        public static final class DualLauncher {
            // Velocity tolerance for considering the launcher "at speed" (rps)
            public static final double kVelocityToleranceRPS = 5.0;

            // Default target velocity (rps) — typically overridden by commands/state handler
            public static final double kDefaultTargetRPS = 40.0;

            // Current limits (amps) — protects motors and prevents brownout
            public static final Current kSupplyCurrentLimit = Amps.of(40);
            public static final Current kStatorCurrentLimit = Amps.of(80);

            // Bottom roller (large flywheel) — Clockwise Positive
            public static final class BottomRoller {
                public static final double kS = 0.25;  // Static friction (volts)
                public static final double kV = 0.12;  // Velocity feedforward (volts per rps)
                public static final double kA = 0.0;   // Acceleration feedforward (volts per rps^2)
                public static final double kP = 0.4;   // Proportional gain
                public static final double kI = 0.0;   // Integral gain
                public static final double kD = 0.0;   // Derivative gain
            }

            // Top roller (smaller contact wheels) — CounterClockwise Positive
            public static final class TopRoller {
                public static final double kS = 0.25;  // Static friction (volts)
                public static final double kV = 0.12;  // Velocity feedforward (volts per rps)
                public static final double kA = 0.0;   // Acceleration feedforward (volts per rps^2)
                public static final double kP = 0.4;   // Proportional gain
                public static final double kI = 0.0;   // Integral gain
                public static final double kD = 0.0;   // Derivative gain
            }
        }

        // 3D Transform (placeholder - measure from CAD)
        public static final Transform3d kRobotToShooter =
            new Transform3d(
                new Translation3d(Inches.of(-5.75), Inches.of(5.534), Inches.of(19.874)),  // Placeholder: 0.5m height Was: 9.427
                new Rotation3d(Rotations.zero(), Rotations.zero(), Rotations.of(-kTurretEncoderOffset))                    // No rotation offset
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
            // map.put(5.334, 34.0 * 60.0);
            // map.put(3.581, 27.5 * 60.0);
            // map.put(4.470, 29.5 * 60.0);
            // map.put(2.515, 23.5 * 60.0);
            // map.put(2.690, 24.9 * 60.0);
            // map.put(2.97, 24.9 * 60.0);
            // map.put(3.251, 26.0 * 60.0);
            map.put(3.4798, 26.6 * 60.0);
            map.put(4.572, 29.4 * 60.0);
            map.put(2.94, 25.25 * 60.0);
            map.put(2.5, 24.2 * 60.0);
            map.put(4.01, 27.55 * 60.0);

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

    public static final class FeederConstants
    {
        // Max voltage output for feeder motor (for brownout protection)
        public static final double kMaxFeederVoltage = 8.0;

        public static final double kFeederIdleVoltage = 0.0;
        public static final double kFeederRunningVoltage = -5.75;
        public static final double kFeederWaitingVoltage = -2.0;
    }


    public static final class ExampleConstants
    {
        public static final double kExampleSpeed = 0.5;  //percentage based where 1.0 is max power and 0.0 is minimum
    }

    public static final class IntakeConstants
    {
        public static enum IntakePosition
        {
            /** Set the intake angle to 0.271 intake rotations */
            GROUND(0.271),
            /** Set the intake angle to 0.213 intake rotations */
            COMPACTING(0.067),
            /** Set the intake angle to 0 rotations (zero position) */
            ZERO(0.0);
            

            /**
             * The target position for the intake in mechanism rotations (not motor rotations). Positive is up, negative is down.
             */
            public final double rotations;
            IntakePosition(double rotations)
            {
                this.rotations = rotations;
            }
        }

        // PID Constants
        public static final double kPositionerKp = 28.67; //20
        public static final double kPositionerKi = 6.0;
        public static final double kPositionerKd = 1.0;
        public static final double kPositionerKs = 0.0;
        public static final double kPositionerKv = 0.0;
        public static final double kPositionerKa = 0.0;
        public static final double kPositionerKG = 0.0;

        // Positioner voltage limits
        public static final double kPositionerPeakForwardVoltage = 4.0;
        public static final double kPositionerPeakReverseVoltage = -4.0;

        // Positioner Motion Magic configuration
        public static final double kPositionerMMCruiseVelocity = 2.0;      // Rotations per second
        public static final double kPositionerMMAcceleration = 25.0;       // Rotations per second squared
        public static final double kPositionerMMJerk = 50.0;               // Rotations per second cubed
        public static final double kPositionerMMExpoKV = 0.12;
        public static final double kPositionerMMExpoKA = 0.1;

        // Positioner current limits
        public static final double kPositionerSupplyCurrentLimit = 60;
        public static final double kPositionerStatorCurrentLimit = 80;

        // Positioner feedback configuration
        public static final double kPositionerSensorToMechanismRatio = 16.4;
        public static final double kPositionerGainSchedulerErrorThreshold = 0.04;
        public static final double kPositionerPositionTolerance = 0.02;    // Rotations

        // Intake roller current limits
        public static final double kIntakeSupplyCurrentLimit = 40;
        public static final double kIntakeStatorCurrentLimit = 60;

        // Intake compact command oscillation
        public static final double kIntakeCompactSwitchIntervalSeconds = 1.2;

        public static final double kIntakeMotorSpeed = 0.5;
        public static final double kPositionerMotorSpeed = 0.5;

        public static final double kPositionerMotorMinPosition = 0.5;
        public static final double kPositionMotorMaxPosition = 0.5;

        public static final double kMaxIntakeVoltage = 8.0;

        public static final double kIntakeFullVoltage = -5.5;
        public static final double kIntakeStationaryVoltage = -4.0;
        public static final double kIntakeIdleVoltage = 0.0;

        public static final double kChassisSpeedRollerFF = 0.66; // Volts of output per m/s of velocity in the intake's direction
    }

    public static final String kCANivoreName = "SwerveCANivore";

    /** The file that is used for system instantiation at runtime */
    public static final String SUBSYSTEMFILE = "Subsystems.json";
}

