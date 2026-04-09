package frc.robot.subsystems.launcher.moveandshoot;

import static frc.robot.Konstants.LauncherConstants.kConvergenceThresholdMeters;
import static frc.robot.Konstants.LauncherConstants.kFixedLaunchAngle;
import static frc.robot.Konstants.LauncherConstants.kMaxIterations;
import static frc.robot.Konstants.LauncherConstants.kMaxRangeMeters;
import static frc.robot.Konstants.LauncherConstants.kMinRangeMeters;
import static frc.robot.Konstants.LauncherConstants.kPhaseDelaySeconds;

import static frc.robot.Konstants.TurretConstants.kTurretBaseLeadTimeSeconds;
import static frc.robot.Konstants.TurretConstants.kTurretCoordinateOffset;
import static frc.robot.Konstants.TurretConstants.kTurretMaxLeadScale;
import static frc.robot.Konstants.TurretConstants.kTurretMinLeadScale;
import static frc.robot.Konstants.TurretConstants.kTurretReferenceYawVelocity;

import org.littletonrobotics.junction.Logger;

import static frc.robot.Konstants.TargetPointConstants.TargetPoint.kOperatorControlled;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Konstants.LauncherConstants;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.launcher.mechanisms.SK26DualLauncher;
import frc.robot.subsystems.launcher.moveandshoot.ShotCalculationStrategy.Range;
import frc.robot.subsystems.turret.SK26Turret;
import frc.lib.utils.Field;
import frc.lib.utils.FieldConstants.LinesHorizontal;

public class ShootingCoordinator {
    SKSwerve drive;
    SK26DualLauncher launcher;
    SK26Turret turret;

    private final ShotCalculator shotCalculator = createShotCalculator();

    ShotParameters currentShot = new ShotParameters(
        RotationsPerSecond.zero(), 
        Degrees.zero(), 
        Degrees.zero(), 
        DegreesPerSecond.zero(), 
        Translation3d.kZero, 
        Translation3d.kZero, 
        Meters.zero(), 
        Seconds.zero(), 
        shotCalculator.getStrategy().getName(), 
        0, 
        false,
        false, 
        0.0);

    private final LauncherTuning tuning;

    // Hysteresis for shuttle target selection to prevent oscillation near center line
    // When true, mirror the shuttle point's Y coordinate (aim at the opposite side)
    private boolean shouldMirrorShuttleTarget = false;
    private static final double SHUTTLE_MIRROR_THRESHOLD = 0.3; // meters past center to trigger mirror
    private static final double SHUTTLE_UNMIRROR_THRESHOLD = 0.3; // meters back from center to un-mirror

    public ShootingCoordinator(
        SK26DualLauncher launcher,
        SK26Turret turret,
        SKSwerve drive
    ) {
        this.launcher = launcher;
        this.turret = turret;
        this.drive = drive;
        this.tuning = launcher.getLauncherTuning();

        System.out.println("[ShootingCoordinator] Initialized");
    }

    // public MoveAndShootSystem() {
    //     this.launcher = new SK26Launcher();
    //     this.turret = new SK26Turret();
    //     this.tuning = launcher.getLauncherTuning();
    //     this.coordinator = new ShootingCoordinator(launcher, turret, shotCalculator);
    // }

    /* ======= Shooting Coordination Commands ======= */

    // public Command scoreStationary() {
    //     return Commands
    // }

    public Command scoreMoving() {
        return Commands.parallel(
            // Continuously update shot calculation using hex rim aim point
            Commands.run(() -> updateShotCalculation(
                HubTargetingSystem.getAimPoint(drive.getRobotPose(), !Field.isBlue())
            )),
            // Continuously run flywheel at calculated speed
            launcher.runVelocityCommand(() -> currentShot.flywheelSpeed()),
            // Continuously aim turret with lead angle compensation
            Commands.run(() -> {
                turret.setAngleDegrees(MathUtil.inputModulus(calculateAdjustedTurretAngle(currentShot).in(Degrees), -180, 180));
            }, turret)
        ).withName("ScoreAndMoveCommand");
    }

    // public Command shuttleStationary() {

    // }

    public Command shuttleMoving() {
        return Commands.parallel(
            // Continuously update shot calculation with hysteresis for target selection
            Commands.run(() ->
                updateShotCalculation(getEffectiveShuttleTarget(kOperatorControlled.point.getTargetPoint()))
            ),
            // Continuously run flywheel at calculated speed
            launcher.runVelocityCommand(() -> currentShot.flywheelSpeed()),
            Commands.run(() -> {
                turret.setAngleDegrees(MathUtil.inputModulus(calculateAdjustedTurretAngle(currentShot).in(Degrees), -180, 180));
            }, turret)
        ).withName("ShuttleAndMoveCommand");
    }

    /**
     * Updates the shuttle mirror state with hysteresis and returns the effective target.
     * 
     * <p>When the robot crosses far enough past the center line (Y axis), it should aim
     * at the mirrored shuttle point to avoid shooting through the net. Hysteresis prevents
     * oscillation when hovering near the center line.
     * 
     * @param shuttlePoint The operator-controlled shuttle point
     * @return The effective target, potentially mirrored across the center line
     */
    private Translation2d getEffectiveShuttleTarget(Translation2d shuttlePoint) {
        double robotY = drive.getRobotPose().getTranslation().getY();
        double shuttleY = shuttlePoint.getY();
        
        // Determine which side of center the shuttle point is on
        boolean shuttleOnPositiveSide = shuttleY > LinesHorizontal.center;
        
        // Calculate distance from center line (positive = towards positive Y, negative = towards negative Y)
        double distanceFromCenter = robotY - LinesHorizontal.center;
        
        // Apply hysteresis for mirror state changes
        // Mirror when robot crosses SHUTTLE_MIRROR_THRESHOLD past center (opposite side from shuttle)
        // Unmirror when robot returns SHUTTLE_UNMIRROR_THRESHOLD onto shuttle's side of center
        if (shuttleOnPositiveSide) {
            // Shuttle is on positive Y side
            // Mirror when robot is too far on negative side
            if (distanceFromCenter < -SHUTTLE_MIRROR_THRESHOLD) {
                shouldMirrorShuttleTarget = true;
            }
            // Unmirror when robot returns to positive side with buffer
            else if (distanceFromCenter > SHUTTLE_UNMIRROR_THRESHOLD) {
                shouldMirrorShuttleTarget = false;
            }
        } else {
            // Shuttle is on negative Y side
            // Mirror when robot is too far on positive side
            if (distanceFromCenter > SHUTTLE_MIRROR_THRESHOLD) {
                shouldMirrorShuttleTarget = true;
            }
            // Unmirror when robot returns to negative side with buffer
            else if (distanceFromCenter < -SHUTTLE_UNMIRROR_THRESHOLD) {
                shouldMirrorShuttleTarget = false;
            }
        }
        
        // Return mirrored or original target
        if (shouldMirrorShuttleTarget) {
            double mirroredY = 2 * LinesHorizontal.center - shuttleY;
            Translation2d mirroredTarget = new Translation2d(shuttlePoint.getX(), mirroredY);
            return mirroredTarget;
        }
        return shuttlePoint;
    }

    /* ======= Subsystem getters ======= */

    public SKSwerve getDrive() {
        return drive;
    }

    public SK26DualLauncher getLauncher() {
        return launcher;
    }

    public SK26Turret getTurret() {
        return turret;
    }

    /* ======= Status retrieval & shot calculator methods ======= */

    /**
     * Get the most recently calculated shot parameters.
     * @return The current shot parameters, or null if not yet calculated
     */
    public ShotParameters getCurrentShot() {
        return currentShot;
    }

    /**
     * @return The shot calculator used by this coordinator
     */
    public ShotCalculator getShotCalculator() {
        return shotCalculator;
    }
    
    /**
     * Check if the current shot is valid (within range, converged).
     * @return true if the shot is valid and can be taken
     */
    public boolean isValidShot() {
        return currentShot != null && currentShot.validShot() && currentShot.converged();
    }

    /**
     * Check if the launcher is at the target speed.
     * @return true if the flywheel is at the required RPM
     */
    public boolean isLauncherReady() {
        return launcher.atTargetVelocity();
    }

    /**
     * Check if the turret is aimed at the target.
     * 
     * @return true if the turret is at the target angle
     */
    public boolean isTurretReady() {
        return turret.atTarget();
    }

    /**
     * Check if everything is ready to shoot.
     * 
     * @return true if shot is valid, launcher is at speed, and turret is aimed
     */
    public boolean isReadyToShoot() {
        return isValidShot() && isLauncherReady() && isTurretReady();
    }

    /**
     * Calculates the turret angle with lead angle compensation applied.
     * 
     * <p>Uses Option 3 framework (velocity-dependent lookahead) with configurable scaling.
     * With default values (minScale=1.0, maxScale=1.0), behaves like Option 2 (fixed lookahead).
     * 
     * <p>Formula:
     * <pre>
     * scaleFactor = clamp(|yawVelocity| / referenceVelocity, minScale, maxScale)
     * adaptiveLeadTime = baseLeadTime × scaleFactor
     * leadAngle = yawVelocity × adaptiveLeadTime
     * fieldRelativeAngle = baseYaw + leadAngle
     * robotRelativeAngle = fieldRelativeAngle - robotHeading - (robotYawVelocity × adaptiveLeadTime)
     * </pre>
     *
     * @param shot The shot parameters containing base yaw and yaw velocity (field-relative)
     * @return The adjusted turret angle in degrees with lead compensation applied (robot-relative)
     */
    public Angle calculateAdjustedTurretAngle(ShotParameters shot) {
        double baseAngleDeg = shot.launcherYaw().in(Degrees);
        double yawVelocityDegPerSec = shot.launcherYawVelocity().in(DegreesPerSecond);
        
        // Calculate velocity-dependent scale factor
        double velocityMagnitude = Math.abs(yawVelocityDegPerSec);
        double scaleFactor = MathUtil.clamp(
            velocityMagnitude / kTurretReferenceYawVelocity,
            kTurretMinLeadScale,
            kTurretMaxLeadScale
        );
        
        // Apply adaptive lead time
        double adaptiveLeadTime = kTurretBaseLeadTimeSeconds * scaleFactor;
        double leadAngleDeg = yawVelocityDegPerSec * adaptiveLeadTime;
        
        // Field-relative angle with lead compensation
        double fieldRelativeAngleDeg = baseAngleDeg + leadAngleDeg;
        
        // Convert from field-relative to robot-relative by subtracting robot heading
        double robotHeadingDeg = drive.getRobotPose().getRotation().getDegrees();
        double robotRelativeAngleDeg = fieldRelativeAngleDeg - robotHeadingDeg;
        
        // Compensate for robot rotation: predict where the robot will be rotated to
        // and pre-adjust the turret angle to counteract it
        double robotYawVelocityDegPerSec = Math.toDegrees(drive.getVelocity(false).omegaRadiansPerSecond);
        double robotRotationCompensationDeg = robotYawVelocityDegPerSec * adaptiveLeadTime;
        robotRelativeAngleDeg -= robotRotationCompensationDeg;
        
        // Convert from standard robot-relative (0° = front) to turret coordinates (0° = left, +90° = front)
        // turretAngle = robotRelativeAngle - kTurretCoordinateOffset
        double turretAngleDeg = robotRelativeAngleDeg - kTurretCoordinateOffset;
        
        return Degrees.of(turretAngleDeg);
    }

    /**
     * Calculate shot parameters for the current robot state.
     * Call this every cycle to get updated parameters.
     * 
     * @param robotPose Current robot pose from odometry/vision
     * @param robotVelocity Current robot velocity from drive subsystem
     * @param targetPosition Field position of the target (e.g., speaker)
     */
    public void updateShotCalculation(
        Pose2d robotPose,
        ChassisSpeeds robotVelocity,
        Translation2d targetPosition
    ) {
        Logger.recordOutput("LaunchingTarget", targetPosition);
        // Calculate new shot parameters
        currentShot = shotCalculator.calculate(
            robotPose,
            robotVelocity,
            targetPosition,
            Timer.getFPGATimestamp()
        );
        Logger.recordOutput("EffectiveLaunchingTarget", drive.getRobotPose().getTranslation().plus(currentShot.effectiveTarget().toTranslation2d()));
        
        // Publish telemetry for debugging
        tuning.publishTelemetry(currentShot);
    }

    public void updateShotCalculation(
        Pose2d robotPose,
        ChassisSpeeds robotVelocity,
        Translation3d targetPosition
    ) {
        Logger.recordOutput("LaunchingTarget", targetPosition.toTranslation2d());
        currentShot = shotCalculator.calculate(
            robotPose, 
            robotVelocity, 
            targetPosition, 
            Timer.getFPGATimestamp());
        Logger.recordOutput("EffectiveLaunchingTarget", drive.getRobotPose().getTranslation().plus(currentShot.effectiveTarget().toTranslation2d()));
        
        tuning.publishTelemetry(currentShot);
    }

    public void updateShotCalculation(Translation3d targetPosition) {
        updateShotCalculation(drive.getRobotPose(), drive.getVelocity(true), targetPosition);
    }

    public void updateShotCalculation(Translation2d targetPosition) {
        updateShotCalculation(drive.getRobotPose(), drive.getVelocity(true), targetPosition);
    }

    public Command updateShotCalculationCommand(Translation2d targetPosition) {
        return Commands.run(() -> updateShotCalculation(targetPosition));
    }

    public Command updateShotCalculationCommand(Translation3d targetPosition) {
        return Commands.run(() -> updateShotCalculation(targetPosition));
    }


    /* ======= Factories for shot calculation ======= */

    /**
     * Creates the ShotCalculator that coordinates everything.
     * 
     * The calculator handles:
     *   1. Phase delay compensation (predict future robot position)
     *   2. Robot-to-shooter transformation (where is the shooter on the robot?)
     *   3. Delegating to the strategy for actual calculation
     */
    public static ShotCalculator createShotCalculator() {
        return new ShotCalculator.Builder()
            .withStrategy(createShotStrategy())
            .withRobotToShooterTransform(LauncherConstants.kRobotToShooter)
            .withPhaseDelay(kPhaseDelaySeconds)
            .build();
    }

    /** 
     * Creates the InterpolatedShotStrategy with characterization data.
     * 
     * The strategy uses:
     *   - Distance (meters) → Flywheel RPM lookup table
     *   - Kinematic calculation for time of flight: t = d / (v × cos(θ) × slipRatio)
     */
    public static InterpolatedShotStrategy createShotStrategy() {
        return new InterpolatedShotStrategy.Builder()
            .withFlywheelSpeedMap(LauncherConstants.createFlywheelSpeedMap())
            .withWheelRadius(LauncherConstants.kWheelRadius)             // For kinematic ToF calculation
            .withSlipRatio(LauncherConstants.kSlipRatio)                 // Ball velocity / wheel velocity compensation
            .withFixedLaunchAngle(kFixedLaunchAngle)
            .withMaxIterations(kMaxIterations)                           // Max lookahead iterations
            .withConvergenceThreshold(kConvergenceThresholdMeters)    // Stop when distance converges within kConvergenceThresholdMeters
            .withValidRange(new Range(kMinRangeMeters, kMaxRangeMeters))  // Valid shooting range
            .build();
    }
}
