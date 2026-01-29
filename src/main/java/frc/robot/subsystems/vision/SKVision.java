package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Konstants.VisionConstants.kAprilTagFieldLayout;
import static frc.robot.Konstants.VisionConstants.kAprilTagPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.Field;
import frc.lib.utils.Trio;
import frc.lib.vision.Limelight;
import frc.lib.vision.Limelight.IMUMode;
import frc.lib.vision.LimelightHelpers.RawFiducial;
import frc.robot.Robot;
import frc.robot.subsystems.drive.SKSwerve;
import static frc.robot.RobotContainer.m_field;

public class SKVision extends SubsystemBase {
    // Swerve reference is needed in order to get drivetrain information like pose and velocity
    private SKSwerve m_swerve;

    // Declare your limelights here and write their hostnames in a comment next to each limelight
    /* Example:
    public final Limelight rightLL = new Limelight(VisionConfig.RIGHT_CONFIG); // limelight-front
    */
    public final Limelight frontLL = new Limelight(VisionConfig.FRONT_CONFIG); // limelight-front
    
    // Array of all limelights
    public final Limelight[] allLimelights = {frontLL}; 
    // Limelights for pose estimation; order them from most used with best view to least used with worst view
    public final Limelight[] poseLimelights = {frontLL}; 
    
    public List<Integer> tagIDsInView = new ArrayList<Integer>();
    public List<Pose3d> tagLOSTransforms = new ArrayList<Pose3d>();
    private StructArrayPublisher<Pose3d> tagLOSPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("VisibleTargetPoses", Pose3d.struct).publish();

    // Boolean stating whether or not limelight poses are being integrated with actual
    public boolean isIntegrating = false; 

    // Provides a boolean as to whether or not the Limelights are "driving" the robot with their measurements
    public boolean isDriving = false; 

    // Creates an ArrayList to store estimated vision poses during autonomous
    public ArrayList<Trio<Pose3d, Pose2d, Double>> multiCamPoses = new ArrayList<Trio<Pose3d, Pose2d, Double>>();

    // Data extraction record (groups related vision measurement data)
    // Package-private for testing
    record VisionMeasurement(
        double timeStamp,
        double targetSize,
        Pose3d botpose3D,
        Pose2d botposeMT1,
        Pose2d botposeMT2,
        RawFiducial[] tags,
        boolean multiTags,
        ChassisSpeeds robotSpeed,
        double poseDifference,
        double highestAmbiguity
    ) {}

    public SKVision(Optional<SKSwerve> m_swerve) {
        this.m_swerve = m_swerve.orElseThrow(
            () -> new IllegalArgumentException("SKSwerve is required for SKVision")
        );

        startupLimelights();
    }

    @Override
    public void periodic() {
        tagIDsInView.clear();
        tagLOSTransforms.clear();

        for(Limelight ll : poseLimelights) {
            ll.setRobotOrientation(m_swerve.getRobotRotation().getDegrees());
            scanForTags(ll);
        }

        SmartDashboard.putData("Vision", this);
        telemeterizeTagLOS();

        /* The secret sauce: */
        estimatePose();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty(
            "Vision Driving", 
            () -> isDriving, 
            null);
        builder.addStringProperty(
            "ResetPoseToVision Status", 
            () -> resetPoseToVisionLog, 
            null);
        
        addStdDevsToBuilder(builder);
        addLimelightsToBuilder(builder);
    }

    private void addLimelightsToBuilder(SendableBuilder builder) {
        for(Limelight ll : allLimelights) {
            builder.addStringProperty(
                ll.getName() + " Status", 
                () -> ll.getLogStatus(), 
                null);
        }
    }

    private void addStdDevsToBuilder(SendableBuilder builder) {
        builder.addDoubleProperty(
            "StdDevs/Vision Std Dev X", 
            () -> VisionConfig.VISION_STD_DEV_X, 
            null);
        builder.addDoubleProperty(
            "StdDevs/Y", 
            () -> VisionConfig.VISION_STD_DEV_Y,
            null);
        builder.addDoubleProperty(
            "StdDevs/Theta", 
            () -> VisionConfig.VISION_STD_DEV_THETA,
            null);
    }

    private void startupLimelights() {
        for(Limelight ll : allLimelights) {
            if(!ll.isAttached()) {
                ll.setLogStatus("Disabled");
            }
            else {
                //ll.setLogStatus("Idle");
            }
            ll.setIMUMode(IMUMode.EXTERNAL); // For Limelight 4s only; this makes sure that we're only using the external IMU for Megatag 2
            ll.setLEDMode(false); // Turns off LED lights on startup
        }
    }

    /**
     * Scans a limelight for visible AprilTags and adds their IDs to an array for later usage.
     * @param ll The limelight to scan for tags
     */
    private void scanForTags(Limelight ll) {
        if(ll.getLimelightPipeline() == kAprilTagPipeline) {
                if(ll.targetInView()) {
                    for(RawFiducial tag : ll.getRawFiducial()) {
                        tagIDsInView.add(tag.id);
                    }
                }
        }
    }

    /**
     * Sends the poses of all visible tags to NetworkTables for logging and telemetry purposes.
     */
    private void telemeterizeTagLOS() {
        for(int id : tagIDsInView) {
            kAprilTagFieldLayout.getTagPose(id).ifPresent(targetPose -> {
                tagLOSTransforms.add(
                    targetPose
                );
            });
        }

        tagLOSPublisher.set(tagLOSTransforms.toArray(Pose3d[]::new));
    }

    /**
     * Adds the estimated pose of a camera into an array to be stored for later usage.
     * <p>
     * Always use this within a for-loop of all cameras you wish to get the estimated pose with
     * and then pass the current camera into the method.
     * @param ll The camera to estimate the robot pose with
     */
    private void updatePoseMultiCam(Limelight ll) {
        // Sets the robot's yaw for use with MEGATAG2 right before integrating with estimator
        ll.setRobotOrientation(m_swerve.getRobotRotation().getDegrees());

        Pose3d botPose3d = ll.getRawPose3d(); // Gets the 3D pose of the limelight on the robot using the position offsets
        Pose2d megaPose2d = ll.getMegaPose2d(); // Gets the estimated pose of the robot using MegaTag2 objects
        double timestamp = ll.getRawPoseTimestamp(); // Timestamp of when the pose was collected

        Pose2d estimatedRobotPose = // Uses limelight (effectively swerve) rotation value instead of estimated rotation value
            new Pose2d(megaPose2d.getTranslation(), megaPose2d.getRotation()); 
        
        multiCamPoses.add(Trio.of(botPose3d, estimatedRobotPose, timestamp)); // Adds both poses into the auto pose feeder

    }

    private void updatePoseAutonomous() {
        if(!DriverStation.isAutonomousEnabled()) {
            return;
        }
        for(Limelight ll : poseLimelights) {
            if(!ll.targetInView()) {
                continue;
            }
            updatePoseSingleCam(ll);
        }
    }

    private void updatePoseSingleCam(Limelight ll) {
        try {
            // Set the integrating status to false before checking if any limelight is integrating
            isIntegrating = false;

            // Feeds the limelight's robot orientation for use with the MEGATAG2 algorithm
            ll.setRobotOrientation(m_swerve.getRobotRotation().getDegrees());

            // Add the input into the pose estimator
            addFilteredLimelightInput(ll);
            
            // A limelight's integrating status is determined by if it's received a valid status to integrate its pose
            isIntegrating |= ll.getConfig().isIntegrating();
        }
        catch (Exception e) {
            DriverStation.reportWarning("Vision: Attempted to access nonexistent vision pose!", false);
        }
    }

    private void updatePoseTeleop() {
        if (!DriverStation.isTeleopEnabled() && !DriverStation.isDisabled()) {
            return;
        }

        Limelight bestLL = getBestLimelight();
        for(Limelight ll : poseLimelights) {
            if (!ll.getName().equals(bestLL.getName())) {
                ll.sendInvalidStatus("Rejected: Not best Limelight");
            }
        }

        updatePoseSingleCam(bestLL);    
    }

    public void estimatePose() {
        // Make sure robot orientation is correct and applied to all pose limelights before updating pose
        /**
        Autonomous pose updater:
        
        For each limelight that has a tag in view, send it to be aggregated into the current pose estimate, alongside other
        recent poses. To update the drivetrain's pose to the vision measurements, use the command for autonResetPoseToVision().
        */
        updatePoseAutonomous();

        /*
        Teleop/disabled pose updater:

        Instead of allowing all limelights to feed data into the SwervePoseEstimator, it scores each limelight that has a
        tag in view based on the tag's proximity to the bot and the number of tags seen. Only then does it feed a pose measurement 
        into the pose estimator. The pose can still be rejected for being too erroneous, but scoring each camera significantly
        reduces the chances of a pose estimate being rejected.
        */
        updatePoseTeleop();
    }

    public Limelight getBestLimelight() {
        Limelight bestLimelight = poseLimelights[0]; // Default limelight to consider "best".
        double bestScore = 0;
        for(Limelight LL : poseLimelights) {
            double score = 0;
            score += LL.getTagCountInView() * VisionConfig.Thresholds.TAG_COUNT_WEIGHT;
            score += LL.getTargetSize(); // Range: 1-100

            if(score > bestScore) {
                bestScore = score;
                bestLimelight = LL;
            }
        }

        return bestLimelight;
    }

    public void forcePoseToVision() {
        if(Robot.isSimulation()) {
            return;
        }
        Limelight ll = getBestLimelight();
        if(ll == null) {
            return;
        }
        if(!ll.targetInView()) {
            return;
        }

        m_swerve.resetPose(ll.getRawPose3d().toPose2d());
        ll.setRobotOrientation(m_swerve.getRobotRotation().getDegrees());
        //TODO: if MT2 doesn't work, change it to the line below
        // m_swerve.resetPose(ll.getRawPose3d().toPose2d());
        // m_swerve.resetPose(ll.getMegaPose2d());
    }

    public void autonResetPoseToVision() {
        boolean reject = true;
        boolean firstSuccess = false;
        int batchSize = 5;
        int endIndex = Math.max(0, multiCamPoses.size() - batchSize);
        /* Starting at the most recent auton pose estimation, analyze the next [batchSize]
        poses and see if they can be succesfully added to the robot's pose estimator
        */ 
        for (int i = multiCamPoses.size() - 1; i > endIndex - 1; i--) {
            Trio<Pose3d, Pose2d, Double> poseInfo = multiCamPoses.get(i);
            boolean success =
                    resetPoseToVision(
                            true, poseInfo.getFirst(), poseInfo.getSecond(), poseInfo.getThird());
            if (success) {
                if (i == multiCamPoses.size() - 1) {
                    firstSuccess = true;
                }
                reject = false;
                System.out.println("AutonResetPoseToVision succeeded on " + (multiCamPoses.size() - i) + " try");
                break;
            }
        }

        if (reject) {
            System.out.println(
                    "AutonResetPoseToVision failed after "
                            + batchSize
                            + " of "
                            + multiCamPoses.size()
                            + " possible tries");
            
            // Flash LEDs Red?
        } else {
            if (firstSuccess) {
                // Flash LEDs Green?
            } else {
                // Flash LEDs Orange?
            }
        }
    }

    public void resetPoseToVision() { // Calls resetPoseToVision by passing in Limelight measurements
        Limelight ll = getBestLimelight();
        if(!ll.targetInView()) {
            return;
        }

        Pose3d botPose3d = ll.getRawPose3d();
        ll.setRobotOrientation(botPose3d.toPose2d().getRotation().getDegrees());
        resetPoseToVision(
                ll.targetInView(), botPose3d, ll.getMegaPose2d(), ll.getRawPoseTimestamp());
    }

    private String resetPoseToVisionLog = "Not executed yet..."; // Provides an updatable string for smartdashboard
    /**
     * Set robot pose to vision pose only if LL has good tag reading
     *
     * @return if the pose was accepted and integrated
     */
    public boolean resetPoseToVision(
        boolean targetInView, Pose3d botpose3D, Pose2d megaPose, double poseTimestamp) {
        boolean reject = false;
        if (targetInView) {
            Pose2d botpose = botpose3D.toPose2d();
            // Pose2d robotPose = m_swerve.getRobotPose(); // TODO: Add telemetry for pose before and after integrating vision
            if (Field.poseOutOfField(megaPose)
                    || Math.abs(botpose3D.getZ()) > VisionConfig.Thresholds.MAX_HEIGHT.in(Meters)
                    || (Math.abs(botpose3D.getRotation().getX()) > VisionConfig.Thresholds.MAX_TILT.in(Radians)
                            || Math.abs(botpose3D.getRotation().getY()) > VisionConfig.Thresholds.MAX_TILT.in(Radians))) {
                resetPoseToVisionLog = (
                        "ResetPoseToVision: FAIL || BAD POSE");
                reject = true;
            }
            if (Field.poseOutOfField(botpose3D)) {
                resetPoseToVisionLog = (
                        "ResetPoseToVision: FAIL || OUT OF FIELD");
                reject = true;
            } else if (Math.abs(botpose3D.getZ()) > VisionConfig.Thresholds.MAX_HEIGHT.in(Meters)) {
                resetPoseToVisionLog = (
                        "ResetPoseToVision: FAIL || IN AIR");
                reject = true;
            } else if ((Math.abs(botpose3D.getRotation().getX()) > VisionConfig.Thresholds.MAX_TILT.in(Radians)
                    || Math.abs(botpose3D.getRotation().getY()) > VisionConfig.Thresholds.MAX_TILT.in(Radians))) {
                        resetPoseToVisionLog = (
                        "ResetPoseToVision: FAIL || TILTED");
                reject = true;
            }

            // don't continue
            if (reject) {
                return !reject; // return the success status
            }

            // track STDs - use reset values for forced pose updates
            VisionConfig.VISION_STD_DEV_X = VisionConfig.PoseStdDevs.RESET.xy();
            VisionConfig.VISION_STD_DEV_Y = VisionConfig.PoseStdDevs.RESET.xy();
            VisionConfig.VISION_STD_DEV_THETA = VisionConfig.PoseStdDevs.RESET.theta();
            m_swerve.setVisionMeasurementStdDevs(
                    VecBuilder.fill(
                            VisionConfig.VISION_STD_DEV_X,
                            VisionConfig.VISION_STD_DEV_Y,
                            VisionConfig.VISION_STD_DEV_THETA));

            Pose2d integratedPose = new Pose2d(megaPose.getTranslation(), megaPose.getRotation());
            // m_swerve.getDrivetrain().addVisionMeasurement(integratedPose, poseTimestamp);
            m_swerve.addVisionMeasurement(integratedPose, poseTimestamp);
            // robotPose = m_swerve.getRobotPose(); // get updated pose
            resetPoseToVisionLog = ("ResetPoseToVision: SUCCESS");
            return true;
        }
        return false; // target not in view
    }

    /**
     * Validates basic requirements for vision measurement integration.
     * @param LL The limelight to validate
     * @return true if basic requirements are met, false otherwise
     */
    private boolean validateBasicRequirements(Limelight LL) {
        if (!LL.targetInView()) {
            LL.setTagStatus("no tags");
            LL.sendInvalidStatus("no tag found rejection");
            return false;
        }
        LL.setRobotOrientation(m_swerve.getRobotRotation().getDegrees());
        return true;
    }

    /**
     * Finds the highest ambiguity among all visible tags.
     * Package-private for testing.
     * @param tags Array of raw fiducial tags
     * @param LL Limelight for logging tag statuses
     * @return The highest ambiguity value, or -1 if no tags
     */
    double findHighestAmbiguity(RawFiducial[] tags, Limelight LL) {
        double highestAmbiguity = -1; // Fixed initialization (was 2)
        LL.setTagStatus("");
        for (RawFiducial tag : tags) {
            if (tag.ambiguity > highestAmbiguity) {
                highestAmbiguity = tag.ambiguity;
            }
            LL.setTagStatus(LL.getTagStatus() + "Tag " + tag.id + ": " + tag.ambiguity);
        }
        return highestAmbiguity;
    }

    /**
     * Extracts vision measurement data from a limelight.
     * @param LL The limelight to extract data from
     * @return VisionMeasurement record containing all relevant data
     */
    private VisionMeasurement extractVisionMeasurement(Limelight LL) {
        double timeStamp = LL.getRawPoseTimestamp();
        double targetSize = LL.getTargetSize();
        Pose3d botpose3D = LL.getRawPose3d();
        Pose2d botposeMT1 = botpose3D.toPose2d();
        Pose2d botposeMT2 = LL.getMegaPose2d();
        RawFiducial[] tags = LL.getRawFiducial();
        boolean multiTags = LL.multipleTagsInView();
        ChassisSpeeds robotSpeed = m_swerve.getRobotRelativeSpeeds();
        double poseDifference = m_swerve.getRobotPose().getTranslation().getDistance(botposeMT2.getTranslation());
        double highestAmbiguity = findHighestAmbiguity(tags, LL);

        return new VisionMeasurement(timeStamp, targetSize, botpose3D, botposeMT1, botposeMT2,
                                     tags, multiTags, robotSpeed, poseDifference, highestAmbiguity);
    }

    /**
     * Checks if a vision measurement should be rejected based on quality thresholds.
     * Package-private for testing.
     * @param m The vision measurement to check
     * @param LL Limelight for logging rejection reasons
     * @return true if the measurement should be rejected, false otherwise
     */
    boolean shouldRejectPose(VisionMeasurement m, Limelight LL) {
        if (m.highestAmbiguity() > VisionConfig.Thresholds.MAX_AMBIGUITY) {
            LL.sendInvalidStatus("ambiguity rejection");
            return true;
        }
        if (Field.poseOutOfField(m.botposeMT2())) {
            LL.sendInvalidStatus("bound rejection - pose OOB");
            return true;
        }
        if (Math.abs(m.robotSpeed().omegaRadiansPerSecond) >= VisionConfig.Thresholds.MAX_ROTATION_SPEED.in(RadiansPerSecond)) {
            LL.sendInvalidStatus("rot speed rejection - too fast");
            return true;
        }
        if (Math.abs(m.botpose3D().getZ()) > VisionConfig.Thresholds.MAX_HEIGHT.in(Meters)) {
            LL.sendInvalidStatus("height rejection - in air");
            return true;
        }
        if (Math.abs(m.botpose3D().getRotation().getX()) > VisionConfig.Thresholds.MAX_TILT.in(Radians)
                || Math.abs(m.botpose3D().getRotation().getY()) > VisionConfig.Thresholds.MAX_TILT.in(Radians)) {
            LL.sendInvalidStatus("roll/pitch rejection - tilted");
            return true;
        }
        if (m.targetSize() <= VisionConfig.Thresholds.MIN_SIZE) {
            LL.sendInvalidStatus("size rejection - target too small");
            return true;
        }
        return false;
    }

    /**
     * Determines the appropriate standard deviations for vision measurement integration
     * based on robot state and vision measurement quality.
     * Package-private for testing.
     *
     * @param robotSpeed Current robot chassis speeds
     * @param targetSize Size of the visible target (0-1 range)
     * @param multiTags Whether multiple tags are visible
     * @param highestAmbiguity Highest ambiguity of visible tags
     * @param poseDifference Distance between current pose and vision pose
     * @param ll Limelight to send status messages
     * @return PoseStdDevs with xy and theta standard deviations, or null to reject
     */
    VisionConfig.PoseStdDevs calculateIntegrationStdDevs(
        ChassisSpeeds robotSpeed,
        double targetSize,
        boolean multiTags,
        double highestAmbiguity,
        double poseDifference,
        Limelight ll
    ) {
        // Stationary and very close
        if (Math.hypot(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond) <= VisionConfig.Thresholds.STATIONARY_SPEED.in(MetersPerSecond)
                && targetSize > VisionConfig.Thresholds.VERY_CLOSE_SIZE) {
            ll.sendValidStatus("Stationary close integration");
            return VisionConfig.PoseStdDevs.STATIONARY_CLOSE;
        }

        // Multi-tag integration
        else if (multiTags && targetSize > VisionConfig.Thresholds.SMALL_MULTI_SIZE) {
            if (targetSize > VisionConfig.Thresholds.LARGE_MULTI_SIZE) {
                ll.sendValidStatus("Strong Multi integration");
                return VisionConfig.PoseStdDevs.STRONG_MULTI;
            }
            ll.sendValidStatus("Multi integration");
            return VisionConfig.PoseStdDevs.MULTI_TAG;
        }

        // Close single tag
        else if (targetSize > VisionConfig.Thresholds.CLOSE_SIZE && poseDifference < VisionConfig.Thresholds.CLOSE_POSE_DIFF) {
            ll.sendValidStatus("Close integration");
            return VisionConfig.PoseStdDevs.CLOSE_SINGLE;
        }

        // Proximity integration
        else if (targetSize > VisionConfig.Thresholds.MODERATE_SIZE && poseDifference < VisionConfig.Thresholds.PROXIMITY_POSE_DIFF) {
            ll.sendValidStatus("Proximity integration");
            return VisionConfig.PoseStdDevs.PROXIMITY;
        }

        // Stable integration (low ambiguity)
        else if (highestAmbiguity < VisionConfig.Thresholds.LOW_AMBIGUITY && targetSize >= VisionConfig.Thresholds.STABLE_SIZE) {
            ll.sendValidStatus("Stable integration");
            return VisionConfig.PoseStdDevs.STABLE;
        }

        // Reject - none of the integration criteria met
        else {
            ll.sendInvalidStatus("catch rejection: " + poseDifference + " poseDiff");
            return null;
        }
    }

    /**
     * Integrates a vision measurement with the pose estimator after applying penalties.
     * @param m The vision measurement to integrate
     * @param LL Limelight for logging
     */
    private void integrateVisionMeasurement(VisionMeasurement m, Limelight LL) {
        VisionConfig.PoseStdDevs stdDevs = calculateIntegrationStdDevs(
            m.robotSpeed(), m.targetSize(), m.multiTags(),
            m.highestAmbiguity(), m.poseDifference(), LL
        );

        if (stdDevs == null) {
            return; // Rejected by integration strategy
        }

        // Apply penalties
        double xyStds = stdDevs.xy();
        double degStds = stdDevs.theta();

        if (m.highestAmbiguity() > VisionConfig.Thresholds.HIGH_AMBIGUITY) {
            degStds = Math.min(degStds, VisionConfig.PoseStdDevs.HIGH_AMBIGUITY_PENALTY.theta());
        }

        if (Math.abs(m.robotSpeed().omegaRadiansPerSecond) >= VisionConfig.Thresholds.HIGH_ROTATION_SPEED.in(RadiansPerSecond)) {
            degStds = Math.min(degStds, VisionConfig.PoseStdDevs.HIGH_ROTATION_PENALTY.theta());
        }

        Pose2d integratedPose = new Pose2d(m.botposeMT2().getTranslation(), m.botposeMT2().getRotation());

        VisionConfig.VISION_STD_DEV_X = xyStds;
        VisionConfig.VISION_STD_DEV_Y = xyStds;
        VisionConfig.VISION_STD_DEV_THETA = degStds;

        addVisionMeasurementWithStdDevs(
            integratedPose,
            m.timeStamp(),
            VecBuilder.fill(xyStds, xyStds, degStds)
        );
    }

    /**
     * Main entry point for filtering and integrating a limelight measurement into the pose estimator.
     * Validates the measurement, checks for rejection criteria, and integrates if acceptable.
     * @param LL The limelight to process
     */
    private void addFilteredLimelightInput(Limelight LL) {
        if (!validateBasicRequirements(LL)) {
            return;
        }

        VisionMeasurement measurement = extractVisionMeasurement(LL);

        if (shouldRejectPose(measurement, LL)) {
            return;
        }

        integrateVisionMeasurement(measurement, LL);
    }

    private void addVisionMeasurementWithStdDevs(Pose2d integratedPose, double timeStamp, Vector<N3>stdDevs) {
        // m_swerve.getDrivetrain().setVisionMeasurementStdDevs(stdDevs);

        // m_swerve.getDrivetrain().addVisionMeasurement(integratedPose, timeStamp);

        m_swerve.setVisionMeasurementStdDevs(stdDevs);
        m_swerve.addVisionMeasurement(integratedPose, timeStamp);
    }

    /** If at least one limelight has an accurate pose */
    public boolean hasAccuratePose() {
        for (Limelight limelight : poseLimelights) {
            if (limelight.hasAccuratePose()) return true;
        }
        return false;
    }

    /** Change all LL pipelines to the same pipeline */
    public void setLimelightPipelines(int pipeline) {
        for (Limelight limelight : allLimelights) {
            limelight.setLimelightPipeline(pipeline);
        }
    }

    public static class CommandConfig {
        public double kp = 0;
        public double ki = 0;
        public double kd = 0;
        public double kIZone = 0;
        public double maxVelocity;
        public double maxAcceleration;
        public double tolerance;
        public double maxOutput;
        public double error;
        public int pipelineIndex;
        public Limelight limelight;
        /* For Drive-To commands */
        public double verticalSetpoint;
        public double verticalMaxView;

        /**
         * Configures the pid constants of the controller using PathPlanner Autonomous
         * PIDConstants.
         * @param Constants PIDConstants to set the controller's PID values to.
         */
        public void configKpid(PIDConstants Constants) {
            this.kp = Constants.kP; this.ki = Constants.kI; this.kd = Constants.kD;
        }

        /**
         * Configures the pid constants of the controller using three values
         * @param kp The P value of the controller
         * @param ki The I value of the controller
         * @param kd the D value of the controller
         */
        public void configKpid(double kp, double ki, double kd) {
            this.kp = kp; this.ki = ki; this.kd = kd;
        }

        /**
         * Configures only the P value of the PID controller
         * @param kp The P value of the controller
         */
        public void configKp(double kp) {
            this.kp = kp;
        }

        /**
         * Configures only the I value of the PID controller
         * @param ki The I value of the controller
         */
        public void configKi(double ki) {
            this.ki = ki;
        }

        public void configKiZone(double kIZone) {
            this.kIZone = kIZone;
        }

        /**
         * Configures only the D value of the PID controller
         * @param kd The D value of the controller
         */
        public void configKd(double kd) {
            this.kd = kd;
        }

        /**
         * Configures the Trapezoidal motion profile of the controller
         * @param maxVel The maximum velocity to configure the profile with
         * @param maxAccel The maximum acceleration to configure the profile with
         */
        public void configProfile(double maxVel, double maxAccel) {
            this.maxVelocity = maxVel;
            this.maxAcceleration = maxAccel;
        }

        public void configTolerance(double tolerance) {
            this.tolerance = tolerance;
        }

        public void configMaxOutput(double maxOutput) {
            this.maxOutput = maxOutput;
        }

        public void configError(double error) {
            this.error = error;
        }

        public void configPipelineIndex(int pipelineIndex) {
            this.pipelineIndex = pipelineIndex;
        }

        public void configLimelight(Limelight limelight) {
            this.limelight = limelight;
        }

        public void configVerticalSetpoint(double verticalSetpoint) {
            this.verticalSetpoint = verticalSetpoint;
        }

        public void configVerticalMaxView(double verticalMaxView) {
            this.verticalMaxView = verticalMaxView;
        }

        public CommandConfig() {}
    }

    public static class MultiLimelightCommandConfig extends CommandConfig {
        public Limelight[] limelights;
        public void configLimelights(Limelight... limelights) {
            this.limelights = limelights;
        }
    }
}
