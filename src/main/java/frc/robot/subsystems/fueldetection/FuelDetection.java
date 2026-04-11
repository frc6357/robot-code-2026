package frc.robot.subsystems.fueldetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vision.Limelight.LimelightConfig;
import frc.lib.vision.LimelightHelpers;
import frc.lib.vision.LimelightHelpers.RawDetection;
import frc.robot.subsystems.drive.SKSwerve;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Subsystem that reads Limelight NN detections, projects them onto the field
 * via a pinhole-camera model, maintains a persistent {@link FuelMap}, and logs
 * ball positions for field visualisation.
 *
 * <h2>Projection geometry</h2>
 *
 * The Limelight gives us {@code txnc} / {@code tync} angles (degrees, from
 * principal pixel).  Combined with the known camera height and pitch we can
 * compute the ground-plane distance to the ball, then rotate into field
 * space using the robot heading and camera offset.
 *
 * <ul>
 *   <li>Camera to field projection math ({@link #projectToField})</li>
 *   <li>Persistent tracking with confidence ({@link FuelMap})</li>
 *   <li>Clustering and scoring ({@link FuelScorer})</li>
 * </ul>
 */
public class FuelDetection extends SubsystemBase {

    private static final String PREFIX = "FuelDetection";

    private final String  limelightName;
    private final double  cameraHeightM;
    private final double  cameraForwardM;
    private final double  cameraRightM;
    private final double  cameraPitchDeg;
    private final double  cameraYawDeg;

    private static final double BALL_RADIUS_M   = Units.inchesToMeters(5.91 / 2.0);
    private static final double TARGET_HEIGHT_M  = BALL_RADIUS_M;

    private final FuelMap   fuelMap   = new FuelMap();

    private final SKSwerve swerve;

    // ---------------------------------------------------------------
    //  Constructor
    // ---------------------------------------------------------------

    public FuelDetection(LimelightConfig limelightConfig, Optional<SKSwerve> swerve) {
        this.limelightName  = limelightConfig.getName();
        this.cameraHeightM  = limelightConfig.getUp();
        this.cameraForwardM = limelightConfig.getForward();
        this.cameraRightM   = limelightConfig.getRight();
        this.cameraPitchDeg = limelightConfig.getPitch();
        this.cameraYawDeg   = limelightConfig.getYaw();
        this.swerve         = swerve.orElse(null);
    }

    // ---------------------------------------------------------------
    //  Periodic  (runs every scheduler cycle)
    // ---------------------------------------------------------------

    @Override
    public void periodic() {
        if (swerve == null) return;

        RawDetection[] detections =
                LimelightHelpers.getRawDetections(limelightName);

        List<Translation2d> fieldPoints = new ArrayList<>();
        for (RawDetection d : detections) {
            Translation2d pt = projectToField(d, swerve.getRobotPose(), swerve.getRobotRotation());
            if (pt != null) fieldPoints.add(pt);
        }

        fuelMap.update(fieldPoints.toArray(new Translation2d[0]));
        logFuelPoses();
        logBestCluster();
    }

    // ---------------------------------------------------------------
    //  Pinhole projection  (camera pixels -> field Translation2d)
    // ---------------------------------------------------------------

    private Translation2d projectToField(RawDetection det,
                                         Pose2d robotPose,
                                         Rotation2d heading) {
        double txDeg = det.txnc;
        double tyDeg = det.tync;

        double totalPitchRad = Math.toRadians(cameraPitchDeg + tyDeg);
        if (totalPitchRad >= 0) return null;

        double dz = (cameraHeightM - TARGET_HEIGHT_M) / Math.tan(-totalPitchRad);
        if (dz <= 0) return null;

        double dx = dz * Math.tan(Math.toRadians(txDeg + cameraYawDeg));

        double camForward = dz + cameraForwardM;
        double camRight   = dx + cameraRightM;

        double cos = heading.getCos();
        double sin = heading.getSin();
        double fieldX = robotPose.getX() + camForward * cos - camRight * sin;
        double fieldY = robotPose.getY() + camForward * sin + camRight * cos;

        return new Translation2d(fieldX, fieldY);
    }

    // ---------------------------------------------------------------
    //  Logging helpers
    // ---------------------------------------------------------------

    private void logFuelPoses() {
        List<TrackedFuel> confirmed = fuelMap.getConfirmedFuels();
        Pose2d[] poses2d = new Pose2d[confirmed.size()];
        Pose3d[] poses3d = new Pose3d[confirmed.size()];

        for (int i = 0; i < confirmed.size(); i++) {
            Translation2d t = confirmed.get(i).getFieldPosition();
            poses2d[i] = new Pose2d(t, new Rotation2d());
            poses3d[i] = new Pose3d(t.getX(), t.getY(), BALL_RADIUS_M,
                    new Rotation3d());
        }

        Logger.recordOutput(PREFIX + "/Balls2d", poses2d);
        Logger.recordOutput(PREFIX + "/Balls3d", poses3d);
    }

    private void logBestCluster() {
        if (swerve == null) return;

        Optional<FuelCluster> best =
                FuelScorer.bestCluster(fuelMap.getConfirmedFuels(),
                        swerve.getRobotPose().getTranslation());

        if (best.isPresent()) {
            Translation2d c = best.get().getCentroid();
            Logger.recordOutput(PREFIX + "/BestCluster",
                    new Pose2d(c, new Rotation2d()));
        }
    }

    // ---------------------------------------------------------------
    //  Public accessors
    // ---------------------------------------------------------------

    public FuelMap   getFuelMap()   { return fuelMap; }
}