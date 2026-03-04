package frc.robot.subsystems.fueldetection;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vision.Limelight.LimelightConfig;
import frc.lib.vision.LimelightHelpers;
import frc.lib.vision.LimelightHelpers.RawDetection;
import frc.robot.subsystems.drive.SKSwerve;

/**
 * Subsystem that reads a Limelight NN detector pipeline, projects each
 * detected fuel onto the field using the robot's estimated pose, and
 * maintains a persistent {@link FuelMap}.
 *
 * The limelight to use is supplied via a {@link LimelightConfig} at
 * construction time, so switching cameras is just a matter of passing a
 * different config
 *
 * Projection geometry
 * The Limelight gives us {@code txnc} / {@code tync} angles.
 * Combined with the known camera height and pitch we can
 * compute the ground-plane distance to the ball, then rotate into field space
 * using the robot heading and camera offset.
 *
 * Dashboard keys
 * 
 *   FuelDetection/Num Detections
 *   FuelDetection/Num Tracked
 *   FuelDetection/Fuel 0/X
 *   FuelDetection/Fuel 0/Y
 *   FuelDetection/Fuel 0/Confidence
 *   
 */
public class FuelDetection extends SubsystemBase {

    private static final String PREFIX = "FuelDetection";

    // ---- Camera config  ----
    private final String limelightName;
    /** Camera height above the floor, in meters. */
    private final double cameraHeightM;
    /** Camera forward offset from robot center, in meters. */
    private final double cameraForwardM;
    /** Camera right offset from robot center, in meters. */
    private final double cameraRightM;
    /** Camera pitch above horizontal, in degrees */
    private final double cameraPitchDeg;
    /** Camera yaw from robot forward, in degrees */
    private final double cameraYawDeg;

    // ---- Ball parameters ----
    /** Ball diameter = 5.91 in. The ball center sits at half its diameter. */
    private static final double BALL_RADIUS_M = Units.inchesToMeters(5.91 / 2.0);
    /** Effective target height */
    private static final double TARGET_HEIGHT_M = BALL_RADIUS_M;

    // ---- Dependencies ----
    private final SKSwerve swerve;

    // ---- Detection state ----
    private RawDetection[] detections = new RawDetection[0];
    private int numDetections = 0;
    private boolean targetValid = false;
    private double primaryTx = 0.0;
    private double primaryTy = 0.0;
    private double primaryTa = 0.0;
    private String primaryClass = "";

    // ---- Fuel map ----
    private final FuelMap fuelMap = new FuelMap();

    // ==================== Constructor ====================

    /**
     * @param limelightConfig Limelight configuration describing the camera's
     *                        network name, mounting position, and orientation.
     *                        Swap this to point at any limelight on the robot.
     * @param swerve          Optional swerve reference for field-space projection.
     *
     */
    public FuelDetection(LimelightConfig limelightConfig, Optional<SKSwerve> swerve) {
        this.limelightName  = limelightConfig.getName();
        this.cameraHeightM  = limelightConfig.getUp();
        this.cameraForwardM = limelightConfig.getForward();
        this.cameraRightM   = limelightConfig.getRight();
        this.cameraPitchDeg = limelightConfig.getPitch();
        this.cameraYawDeg   = limelightConfig.getYaw();
        this.swerve = swerve.orElse(null);
        SmartDashboard.putData(PREFIX, this);
    }

    // ==================== Periodic ====================

    @Override
    public void periodic() {
        // ---- 1. Read raw detections from NetworkTables ----
        targetValid  = LimelightHelpers.getTV(limelightName);
        primaryTx    = LimelightHelpers.getTX(limelightName);
        primaryTy    = LimelightHelpers.getTY(limelightName);
        primaryTa    = LimelightHelpers.getTA(limelightName);
        primaryClass = LimelightHelpers.getDetectorClass(limelightName);
        detections   = LimelightHelpers.getRawDetections(limelightName);
        numDetections = detections.length;

        // ---- 2. Project detections to field & update map ----
        if (swerve != null && numDetections > 0) {
            Translation2d[] fieldPositions = new Translation2d[numDetections];
            Pose2d robotPose = swerve.getRobotPose();

            for (int i = 0; i < numDetections; i++) {
                fieldPositions[i] = projectToField(detections[i], robotPose);
            }
            fuelMap.update(fieldPositions);
        } else if (numDetections == 0) {
            // Still need to decay even when nothing is detected
            fuelMap.update(new Translation2d[0]);
        }

        // ---- 3. Post summary telemetry ----
        postTelemetry();

        // ---- 4. Log Pose arrays for AdvantageScope ----
        logFuelPoses();
    }

    // ==================== Projection ====================

    /**
     * Projects a single NN detection onto the field plane.
     *
     * Uses the pinhole-camera ground-plane intersection:
     * 
     *   Vertical angle to target = cameraPitch + tync
     *   Ground distance = (cameraHeight − targetHeight) / tan(vertAngle)
     *   Lateral offset  = groundDist × tan(txnc + cameraYaw)
     *   Convert robot-relative → field-space via the robot pose
     *
     * @param det       Raw detection from the Limelight
     * @param robotPose Current estimated robot pose on the field
     * @return Field-space Translation2d of the ball
     */
    private Translation2d projectToField(RawDetection det, Pose2d robotPose) {
        // Vertical angle from the camera's optical axis to the target (degrees)
        // tync is positive-down from principal pixel, pitch is positive-up
        double vertAngleDeg = cameraPitchDeg - det.tync;
        double vertAngleRad = Math.toRadians(vertAngleDeg);

        // Horizontal angle from camera forward to target
        double horizAngleDeg = det.txnc + cameraYawDeg;
        double horizAngleRad = Math.toRadians(horizAngleDeg);

        // Ground-plane distance straight ahead of the camera
        double heightDiff = cameraHeightM - TARGET_HEIGHT_M;
        double groundDist = heightDiff / Math.tan(vertAngleRad);

        // If the math gives us a negative or absurd distance, reject it
        if (groundDist < 0.05 || groundDist > 8.0) {
            // Return robot's own position as a "bad" marker — the map will
            // reject it via the off-field bounds check
            return robotPose.getTranslation();
        }

        // Robot-relative offset
        double robotRelativeForward = cameraForwardM + groundDist * Math.cos(horizAngleRad);
        double robotRelativeLeft    = cameraRightM   - groundDist * Math.sin(horizAngleRad);
        // Note: cameraRightM is positive-right, WPILib Y is positive-left

        Translation2d robotRelative = new Translation2d(robotRelativeForward, robotRelativeLeft);

        // Rotate into field space and add robot position
        Translation2d fieldPos = robotRelative.rotateBy(robotPose.getRotation())
                                              .plus(robotPose.getTranslation());

        return fieldPos;
    }

    // ==================== AdvantageScope Logging ====================

    /**
     * Logs all tracked fuel positions as Pose2d[] and Pose3d[] arrays so
     * AdvantageScope can render them on the 2D and 3D field views.
     *
     * Pose3d Z is set to the ball's center height (BALL_RADIUS_M) so
     * the ball appears sitting on the floor in the 3D view.
     */
    private void logFuelPoses() {
        List<TrackedFuel> fuels = fuelMap.getTrackedFuels();

        Pose2d[] poses2d = new Pose2d[fuels.size()];
        Pose3d[] poses3d = new Pose3d[fuels.size()];

        for (int i = 0; i < fuels.size(); i++) {
            Translation2d pos = fuels.get(i).getFieldPosition();

            poses2d[i] = new Pose2d(pos, new Rotation2d());
            poses3d[i] = new Pose3d(
                pos.getX(), pos.getY(), BALL_RADIUS_M,
                new Rotation3d()
            );
        }

        Logger.recordOutput("FuelDetection/TrackedFuelPoses2d", poses2d);
        Logger.recordOutput("FuelDetection/TrackedFuelPoses3d", poses3d);
        Logger.recordOutput("FuelDetection/NumTracked", fuels.size());
    }

    // ==================== Dashboard ====================

    /**
     * Posts only summary-level telemetry to SmartDashboard.  Individual fuel
     * entries are logged via AdvantageScope Pose arrays
     */
    private void postTelemetry() {
        SmartDashboard.putBoolean(PREFIX + "/Target Valid",   targetValid);
        SmartDashboard.putNumber( PREFIX + "/Num Detections", numDetections);
        SmartDashboard.putNumber( PREFIX + "/Num Tracked",    fuelMap.size());
        SmartDashboard.putString( PREFIX + "/Primary Class",  primaryClass);
        SmartDashboard.putNumber( PREFIX + "/Primary TX",     primaryTx);
        SmartDashboard.putNumber( PREFIX + "/Primary TY",     primaryTy);
        SmartDashboard.putNumber( PREFIX + "/Primary TA",     primaryTa);
    }

    // ==================== Public API ====================

    /** The persistent fuel map */
    public FuelMap getFuelMap()             { return fuelMap; }

    /** All current NN detector results this frame. */
    public RawDetection[] getDetections()   { return detections; }

    /** Number of objects detected this frame. */
    public int getNumDetections()           { return numDetections; }

    /** Whether the Limelight has any valid target. */
    public boolean isTargetValid()          { return targetValid; }

    /** Primary (best) target TX in degrees. */
    public double getPrimaryTx()            { return primaryTx; }

    /** Primary (best) target TY in degrees. */
    public double getPrimaryTy()            { return primaryTy; }

    /** Primary (best) target area. */
    public double getPrimaryTa()            { return primaryTa; }

    /** Primary detector class name string. */
    public String getPrimaryClass()         { return primaryClass; }

    /** Clears the fuel map */
    public void clearMap()                  { fuelMap.clear(); }

    /**
     * Returns the closest tracked fuel to the given field position, or
     * {@code null} if the map is empty.  Useful for "drive to nearest ball"
     * commands.
     *
     * @param robotPosition The robot's current field-space position
     * @return The nearest {@link TrackedFuel}, or null
     */
    public TrackedFuel getClosestFuel(Translation2d robotPosition) {
        List<TrackedFuel> fuels = fuelMap.getTrackedFuels();
        TrackedFuel closest = null;
        double closestDist = Double.MAX_VALUE;

        for (TrackedFuel f : fuels) {
            double dist = f.getFieldPosition().getDistance(robotPosition);
            if (dist < closestDist) {
                closestDist = dist;
                closest = f;
            }
        }
        return closest;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Target Valid",   this::isTargetValid,    null);
        builder.addIntegerProperty("Num Detections", this::getNumDetections, null);
        builder.addStringProperty("Primary Class",   this::getPrimaryClass,  null);
        builder.addDoubleProperty("Primary TX",      this::getPrimaryTx,     null);
        builder.addDoubleProperty("Primary TY",      this::getPrimaryTy,     null);
        builder.addDoubleProperty("Primary TA",      this::getPrimaryTa,     null);
    }
}
