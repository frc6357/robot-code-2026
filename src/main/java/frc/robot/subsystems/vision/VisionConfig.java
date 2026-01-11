package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.lib.vision.Limelight.LimelightConfig;
import frc.robot.Konstants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.SKVision.MultiLimelightCommandConfig;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
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

    // Standard deviations for vision measurements (in meters and degrees)
    public static double VISION_STD_DEV_X = 0.5; // These are not final because they are sometimes (not often) changed 
    public static double VISION_STD_DEV_Y = 0.5; // on the fly for pose estimating from vision measurements.
    public static double VISION_STD_DEV_THETA = 99999999; // The smaller the number, the stricter the measurement requirements.

    public static final Matrix<N3, N1> visionStdMatrix = // This however, creates a matrix using these standard values to reference for that other 90% of the time
            VecBuilder.fill(VISION_STD_DEV_X, VISION_STD_DEV_Y, VISION_STD_DEV_THETA);
    
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
