package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.vision.Limelight.LimelightConfig;
import frc.robot.Konstants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.SKVision.MultiLimelightCommandConfig;
import frc.robot.Konstants.VisionConstants.FrontLimelight;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Konstants.VisionConstants.*;

public final class VisionConfig {
    /* Example:
    public static final String RIGHT_LL = RightLimelight.kName;
    public static final int RIGHT_TAG_PIPELINE = kAprilTagPipeline;
    public static final LimelightConfig RIGHT_CONFIG = 
                                        new LimelightConfig(RightLimelight.kName) // Yes, it's the same value as [NAME]_LL. Just left it like this to see constructor layout
                                        .withTranslation(RightLimelight.kForward, RightLimelight.kRight, RightLimelight.kUp) // Feeds in the position of the limelight on the bot
                                        .withRotation(RightLimelight.kRoll, RightLimelight.kPitch, RightLimelight.kYaw) // Feeds in rotation of limelight
                                        .withAttached(RightLimelight.kAttached); // Whether or not the limelight is attached to the robot; if false, effectively disables limelight
    */
    public static final String FRONT_LL = FrontLimelight.kName;
    public static final int FRONT_TAG_PIPELINE = kAprilTagPipeline;
    public static final LimelightConfig FRONT_CONFIG = 
                                        new LimelightConfig(FrontLimelight.kName) // Yes, it's the same value as [NAME]_LL. Just left it like this to see constructor layout
                                        .withTranslation(FrontLimelight.kForward, FrontLimelight.kRight, FrontLimelight.kUp) // Feeds in the position of the limelight on the bot
                                        .withRotation(FrontLimelight.kRoll, FrontLimelight.kPitch, FrontLimelight.kYaw) // Feeds in rotation of limelight
                                        .withAttached(FrontLimelight.kAttached); // Whether or not the limelight is attached to the robot; if false, effectively disables limelight


    // Standard deviations for vision measurements (in meters and degrees)
    public static double VISION_STD_DEV_X = 0.5; // These are not final because they are sometimes (not often) changed 
    public static double VISION_STD_DEV_Y = 0.5; // on the fly for pose estimating from vision measurements.
    public static double VISION_STD_DEV_THETA = 99999999; // The smaller the number, the stricter the measurement requirements.

    public static final Matrix<N3, N1> visionStdMatrix = // This however, creates a matrix using these standard values to reference for that other 90% of the time
            VecBuilder.fill(VISION_STD_DEV_X, VISION_STD_DEV_Y, VISION_STD_DEV_THETA);

    // Record for pose standard deviations (groups xy and theta together)
    public record PoseStdDevs(double xy, double theta) {
        public static final PoseStdDevs STATIONARY_CLOSE = new PoseStdDevs(0.1, 0.1);
        public static final PoseStdDevs MULTI_TAG = new PoseStdDevs(0.25, 8.0);
        public static final PoseStdDevs STRONG_MULTI = new PoseStdDevs(0.1, 0.1);
        public static final PoseStdDevs CLOSE_SINGLE = new PoseStdDevs(0.5, 16.0);
        public static final PoseStdDevs PROXIMITY = new PoseStdDevs(2.0, 999999.0);
        public static final PoseStdDevs STABLE = new PoseStdDevs(0.5, 999999.0);
        public static final PoseStdDevs RESET = new PoseStdDevs(0.001, 0.001);

        public static final PoseStdDevs HIGH_AMBIGUITY_PENALTY = new PoseStdDevs(0, 15.0);
        public static final PoseStdDevs HIGH_ROTATION_PENALTY = new PoseStdDevs(0, 15.0);
    }

    // Simple constants for thresholds
    public static final class Thresholds {
        // Physical constraints
        public static final Distance MAX_HEIGHT = Meters.of(0.25);
        public static final Angle MAX_TILT = Degrees.of(5.0);

        // Tag quality
        public static final double MAX_AMBIGUITY = 0.9;
        public static final double HIGH_AMBIGUITY = 0.5;
        public static final double LOW_AMBIGUITY = 0.25;

        // Motion thresholds
        public static final AngularVelocity MAX_ROTATION_SPEED = RadiansPerSecond.of(4 * Math.PI);
        public static final AngularVelocity HIGH_ROTATION_SPEED = RadiansPerSecond.of(0.5);
        public static final LinearVelocity STATIONARY_SPEED = MetersPerSecond.of(0.2);

        // Target size thresholds
        public static final double MIN_SIZE = 0.025;
        public static final double VERY_CLOSE_SIZE = 0.4;
        public static final double CLOSE_SIZE = 0.8;
        public static final double MODERATE_SIZE = 0.1;
        public static final double SMALL_MULTI_SIZE = 0.05;
        public static final double LARGE_MULTI_SIZE = 0.09;
        public static final double STABLE_SIZE = 0.03;

        // Pose difference thresholds
        public static final double CLOSE_POSE_DIFF = 0.5;
        public static final double PROXIMITY_POSE_DIFF = 0.3;

        // Scoring
        public static final double TAG_COUNT_WEIGHT = 100.0;
    }

    // Command configs for vision-based commands
    public static final class DriveToPose extends MultiLimelightCommandConfig {
        private DriveToPose() {
            configKpid(1, 0, 0.001); //1, 0, .001
            configTolerance(0.02);
            configProfile(
                DriveConstants.kMaxSpeed.times(0.55).in(MetersPerSecond), 
                DriveConstants.kMaxSpeed.times(0.55).in(MetersPerSecond) * 2
            ); //55% Max Speed; 2x Acceleration
            configMaxOutput(DriveConstants.kMaxSpeed.times(0.55).in(MetersPerSecond));
            configError(0.01);
            configPipelineIndex(kAprilTagPipeline);
            configLimelights(RobotContainer.m_visionInstance.poseLimelights);
        }

        public static DriveToPose getConfig() {
            return new DriveToPose();
        }
    }

    public static final class RotateToPose extends MultiLimelightCommandConfig {
        private RotateToPose() {
            configKpid(0.006, 0, 0.00015);
            configTolerance(1.5);
            configProfile(
                DriveConstants.kMaxAngularRate.in(DegreesPerSecond) * 0.1, 
                DriveConstants.kMaxAngularRate.in(DegreesPerSecond) * 0.1 * 5); // 10% Angular speed; 5x acceleration
            configMaxOutput(DriveConstants.kMaxAngularRate.in(DegreesPerSecond) * 0.1);
            configError(1);
            configPipelineIndex(kAprilTagPipeline);
            configLimelights(RobotContainer.m_visionInstance.poseLimelights);
        }

        public static RotateToPose getConfig() {
            return new RotateToPose();
        }
    }
}
