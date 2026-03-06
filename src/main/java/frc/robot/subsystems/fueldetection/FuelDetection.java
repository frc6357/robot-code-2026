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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vision.Limelight.LimelightConfig;
import frc.lib.subsystems.PathplannerSubsystem;
import frc.lib.vision.LimelightHelpers;
import frc.lib.vision.LimelightHelpers.RawDetection;
import frc.robot.subsystems.drive.SKSwerve; 

/**
 * Subsystem that reads the Limelight 3 NN detector pipeline, projects each
 * detected fuel (ball) onto the field using the robot's estimated pose, and
 * maintains a persistent, flicker-resistant {@link FuelMap}.
 *
 * <h2>Projection geometry</h2>
 * The Limelight gives us {@code txnc} / {@code tync} angles (degrees, from
 * principal pixel).  Combined with the known camera height and pitch we can
 * compute the ground-plane distance to the ball, then rotate into field space
 * using the robot heading and camera offset.
 *
 * <h2>Dashboard keys</h2>
 * <pre>
 *   FuelDetection/Num Detections      (raw this frame)
 *   FuelDetection/Num Tracked         (persistent map size)
 *   FuelDetection/Fuel 0/X            (field X meters)
 *   FuelDetection/Fuel 0/Y            (field Y meters)
 *   FuelDetection/Fuel 0/Confidence
 *   ...
 * </pre>
 */
public class FuelDetection extends SubsystemBase implements PathplannerSubsystem {

    private static final String PREFIX = "FuelDetection";
    private final String limelightName;

    // ---- Camera mounting (populated from LimelightConfig) ----
    /** Camera height above the floor, in meters. */
    private final double cameraHeightM;
    /** Camera forward offset from robot center, in meters. */
    private final double cameraForwardM;
    /** Camera right offset from robot center, in meters. */
    private final double cameraRightM;
    /** Camera pitch above horizontal, in degrees (positive = tilted up). */
    private final double cameraPitchDeg;
    /** Camera yaw from robot forward, in degrees. */
    private final double cameraYawDeg;

    // ---- Ball parameters ----
    /** Ball diameter = 5.91 in. The ball center sits at half its diameter. */
    private static final double BALL_RADIUS_M = Units.inchesToMeters(5.91 / 2.0);
    /** Effective target height (center of ball on the floor). */
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
    private String jsonDump = "";

    // ---- Fuel map ----
    private final FuelMap fuelMap = new FuelMap();

    // ---- Simulation ----
    /** Non-null only when running in simulation mode. */
    private final SimFuelField simFuelField;

    // Track how many dashboard entries we've written so we can clear stale ones
    private int maxFuelPosted = 0;
    private int maxDetPosted = 0;

    // ==================== Constructor ====================

    /**
     * @param limelightConfig Limelight configuration with camera mounting offsets.
     * @param swerve          Optional swerve reference for field-space projection.
     *                        If empty, raw detection telemetry is still posted but
     *                        the fuel map will not update.
     */
    public FuelDetection(LimelightConfig limelightConfig, Optional<SKSwerve> swerve) {
        this.limelightName = limelightConfig.getName();
        this.cameraHeightM  = limelightConfig.getUp();
        this.cameraForwardM = limelightConfig.getForward();
        this.cameraRightM   = limelightConfig.getRight();
        this.cameraPitchDeg = limelightConfig.getPitch();
        this.cameraYawDeg   = limelightConfig.getYaw();
        this.swerve = swerve.orElse(null);
        this.simFuelField = RobotBase.isSimulation() ? new SimFuelField() : null;
        SmartDashboard.putData(PREFIX, this);

        // Start file-based debug logger
        FuelHuntFileLogger.init();
    }

    // ==================== Periodic ====================

    @Override
    public void periodic() {
        if (simFuelField != null && swerve != null) {
            // ---- SIMULATION MODE: bypass Limelight, inject from SimFuelField ----
            Pose2d robotPose = swerve.getRobotPose();
            Translation2d[] simPositions = simFuelField.getVisibleFuelPositions(robotPose);
            numDetections = simPositions.length;
            targetValid = numDetections > 0;
            fuelMap.update(simPositions);

            Logger.recordOutput("FuelDetection/SimBallsRemaining", simFuelField.getRemainingCount());
        } else {
            // ---- REAL MODE: read from Limelight NN detector ----
            targetValid  = LimelightHelpers.getTV(limelightName);
            primaryTx    = LimelightHelpers.getTX(limelightName);
            primaryTy    = LimelightHelpers.getTY(limelightName);
            primaryTa    = LimelightHelpers.getTA(limelightName);
            primaryClass = LimelightHelpers.getDetectorClass(limelightName);
            detections   = LimelightHelpers.getRawDetections(limelightName);
            numDetections = detections.length;
            jsonDump     = LimelightHelpers.getJSONDump(limelightName);

            // ---- Project detections to field & update map ----
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
        }

        // ---- 3. Post to SmartDashboard ----
        if (simFuelField == null) {
            postRawTelemetry();
        }
        postMapTelemetry();

        // ---- 4. Log Pose arrays for AdvantageScope ----
        logFuelPoses();

        // ---- 5. Log best cluster for AdvantageScope ----
        logBestCluster();

        // ---- 6. File-based debug logger ----
        FuelHuntFileLogger.logCycle();
    }

    // ==================== Projection ====================

    /**
     * Projects a single NN detection onto the field plane.
     *
     * <p>Uses the pinhole-camera ground-plane intersection:
     * <ol>
     *   <li>Vertical angle to target = cameraPitch + tync</li>
     *   <li>Ground distance = (cameraHeight − targetHeight) / tan(vertAngle)</li>
     *   <li>Lateral offset  = groundDist × tan(txnc + cameraYaw)</li>
     *   <li>Convert robot-relative → field-space via the robot pose</li>
     * </ol>
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
            // mostly merge or reject it anyway
            return robotPose.getTranslation();
        }

        // Robot-relative offset (forward, left in WPILib convention)
        double robotRelativeForward = cameraForwardM + groundDist * Math.cos(horizAngleRad);
        double robotRelativeLeft    = cameraRightM   - groundDist * Math.sin(horizAngleRad);
        // Note: CAMERA_RIGHT_M is positive-right, WPILib Y is positive-left

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
     * <p>Pose3d Z is set to the ball's center height (BALL_RADIUS_M) so
     * the ball appears sitting on the floor in the 3D view.
     */
    private void logFuelPoses() {
        List<TrackedFuel> fuels = fuelMap.getConfirmedFuels();

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

    /**
     * Logs the best-scored fuel cluster centroid so it can be visualized
     * in AdvantageScope as a separate marker / ghost robot pose.
     */
    private void logBestCluster() {
        if (swerve == null) return;

        Translation2d robotPos = swerve.getRobotPose().getTranslation();
        java.util.Optional<FuelCluster> best = FuelScorer.bestCluster(
            fuelMap.getConfirmedFuels(), robotPos);

        if (best.isPresent()) {
            Translation2d c = best.get().getCentroid();
            Logger.recordOutput("FuelDetection/BestCluster",
                new Pose2d[] { new Pose2d(c, new Rotation2d()) });
            Logger.recordOutput("FuelDetection/BestClusterCount", best.get().getCount());
        } else {
            Logger.recordOutput("FuelDetection/BestCluster", new Pose2d[0]);
            Logger.recordOutput("FuelDetection/BestClusterCount", 0);
        }
    }

    // ==================== Dashboard ====================

    private void postRawTelemetry() {
        SmartDashboard.putBoolean(PREFIX + "/Target Valid",   targetValid);
        SmartDashboard.putNumber( PREFIX + "/Num Detections", numDetections);
        SmartDashboard.putString( PREFIX + "/Primary Class",  primaryClass);
        SmartDashboard.putNumber( PREFIX + "/Primary TX",     primaryTx);
        SmartDashboard.putNumber( PREFIX + "/Primary TY",     primaryTy);
        SmartDashboard.putNumber( PREFIX + "/Primary TA",     primaryTa);

        for (int i = 0; i < numDetections; i++) {
            String det = PREFIX + "/Det " + i;
            RawDetection d = detections[i];
            SmartDashboard.putNumber(det + "/Class ID",   d.classId);
            SmartDashboard.putNumber(det + "/TX NoCross",  d.txnc);
            SmartDashboard.putNumber(det + "/TY NoCross",  d.tync);
            SmartDashboard.putNumber(det + "/TA",          d.ta);
        }
        for (int i = numDetections; i < maxDetPosted; i++) {
            String det = PREFIX + "/Det " + i;
            SmartDashboard.putNumber(det + "/Class ID",   0);
            SmartDashboard.putNumber(det + "/TX NoCross",  0);
            SmartDashboard.putNumber(det + "/TY NoCross",  0);
            SmartDashboard.putNumber(det + "/TA",          0);
        }
        maxDetPosted = Math.max(maxDetPosted, numDetections);

        SmartDashboard.putString(PREFIX + "/JSON Dump", jsonDump);
    }

    private void postMapTelemetry() {
        List<TrackedFuel> fuels = fuelMap.getConfirmedFuels();
        SmartDashboard.putNumber(PREFIX + "/Num Tracked", fuels.size());

        for (int i = 0; i < fuels.size(); i++) {
            String key = PREFIX + "/Fuel " + i;
            TrackedFuel f = fuels.get(i);
            SmartDashboard.putNumber(key + "/ID",         f.id);
            SmartDashboard.putNumber(key + "/X",          f.getFieldPosition().getX());
            SmartDashboard.putNumber(key + "/Y",          f.getFieldPosition().getY());
            SmartDashboard.putNumber(key + "/Confidence",  f.getConfidence());
            SmartDashboard.putNumber(key + "/Observations", f.getTotalObservations());
        }
        for (int i = fuels.size(); i < maxFuelPosted; i++) {
            String key = PREFIX + "/Fuel " + i;
            SmartDashboard.putNumber(key + "/ID",         -1);
            SmartDashboard.putNumber(key + "/X",          0);
            SmartDashboard.putNumber(key + "/Y",          0);
            SmartDashboard.putNumber(key + "/Confidence",  0);
            SmartDashboard.putNumber(key + "/Observations", 0);
        }
        maxFuelPosted = Math.max(maxFuelPosted, fuels.size());
    }

    // ==================== Public API ====================

    /** The persistent fuel map (for use by commands / auto logic). */
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

    /** Full raw JSON dump from the Limelight. */
    public String getJsonDump()             { return jsonDump; }

    /** Clears the fuel map (useful on mode transitions). Also resets sim balls. */
    public void clearMap() {
        fuelMap.clear();
        if (simFuelField != null) {
            simFuelField.reset();
        }
    }

    /** Returns the sim fuel field, or null if not in simulation. */
    public SimFuelField getSimFuelField() {
        return simFuelField;
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

    @Override
    public void addPathPlannerCommands() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addPathPlannerCommands'");
    }
}
