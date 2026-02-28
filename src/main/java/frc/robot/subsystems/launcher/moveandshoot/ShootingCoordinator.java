package frc.robot.subsystems.launcher.moveandshoot;

import static frc.robot.Konstants.LauncherConstants.kConvergenceThresholdMeters;
import static frc.robot.Konstants.LauncherConstants.kFixedLaunchAngle;
import static frc.robot.Konstants.LauncherConstants.kMaxIterations;
import static frc.robot.Konstants.LauncherConstants.kMaxRangeMeters;
import static frc.robot.Konstants.LauncherConstants.kMinRangeMeters;
import static frc.robot.Konstants.LauncherConstants.kPhaseDelaySeconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Konstants.LauncherConstants;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.launcher.mechanisms.BangBangLauncher;
import frc.robot.subsystems.launcher.moveandshoot.ShotCalculationStrategy.Range;
import frc.robot.subsystems.turret.SK26Turret;

public class ShootingCoordinator {
    SKSwerve drive;
    BangBangLauncher launcher;
    SK26Turret turret;

    private final ShotCalculator shotCalculator = createShotCalculator();

    ShotParameters currentShot;

    private final LauncherTuning tuning;

    public ShootingCoordinator(
        BangBangLauncher launcher,
        SK26Turret turret,
        SKSwerve drive
    ) {
        this.launcher = launcher;
        this.turret = turret;
        this.drive = drive;
        this.tuning = launcher.getLauncherTuning();
    }

    // public MoveAndShootSystem() {
    //     this.launcher = new SK26Launcher();
    //     this.turret = new SK26Turret();
    //     this.tuning = launcher.getLauncherTuning();
    //     this.coordinator = new ShootingCoordinator(launcher, turret, shotCalculator);
    // }

    /* ======= Subsystem getters ======= */

    public SKSwerve getDrive() {
        return drive;
    }

    public BangBangLauncher getLauncher() {
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
        return launcher.isAtGoal();
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
        // Calculate new shot parameters
        currentShot = shotCalculator.calculate(
            robotPose,
            robotVelocity,
            targetPosition,
            Timer.getFPGATimestamp()
        );
        
        // Publish telemetry for debugging
        tuning.publishTelemetry(currentShot);
    }

    public void updateShotCalculation(Translation2d targetPosition) {
        updateShotCalculation(drive.getRobotPose(), drive.getVelocity(true), targetPosition);
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
     * The strategy uses lookup tables (InterpolatingDoubleTreeMap) that map:
     *   - Distance (meters) → Flywheel RPM
     *   - Distance (meters) → Time of Flight (seconds)
     */
    public static InterpolatedShotStrategy createShotStrategy() {
        return new InterpolatedShotStrategy.Builder()
            .withFlywheelSpeedMap(LauncherConstants.createFlywheelSpeedMap())
            .withTimeOfFlightMap(LauncherConstants.createTimeOfFlightMap())
            .withFixedLaunchAngle(kFixedLaunchAngle)
            .withMaxIterations(kMaxIterations)                           // Max lookahead iterations
            .withConvergenceThreshold(kConvergenceThresholdMeters)    // Stop when distance converges within kConvergenceThresholdMeters
            .withValidRange(new Range(kMinRangeMeters, kMaxRangeMeters))  // Valid shooting range
            .build();
    }
}
