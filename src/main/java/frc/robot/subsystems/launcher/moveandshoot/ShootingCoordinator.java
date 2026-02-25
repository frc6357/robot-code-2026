package frc.robot.subsystems.launcher.moveandshoot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.launcher.mechanisms.BangBangLauncher;
import frc.robot.subsystems.turret.SK26Turret;

/**
 * Coordinates shot calculations with launcher and turret subsystems.
 * 
 * <p>This class ties together:
 * <ul>
 *   <li>{@link ShotCalculator} - Calculates shot parameters</li>
 *   <li>{@link BangBangLauncher} - Spins flywheel to target speed</li>
 *   <li>{@link SK26Turret} - Aims at the target</li>
 *   <li>{@link LauncherTuning} - Publishes telemetry</li>
 * </ul>
 * 
 * <p>Usage:
 * <pre>{@code
 * ShootingCoordinator coordinator = new ShootingCoordinator(launcher, turret, calculator);
 * 
 * // Continuous tracking command
 * Command trackCommand = coordinator.aimAndSpinUpCommand(
 *     poseSupplier, velocitySupplier, targetPosition
 * );
 * 
 * // Full shooting sequence
 * Command shootCommand = coordinator.fullShootingSequence(
 *     poseSupplier, velocitySupplier, targetPosition, feedCommand
 * );
 * }</pre>
 */
public class ShootingCoordinator {
    
    private final ShotCalculator shotCalculator;
    private final BangBangLauncher launcher;
    private final SK26Turret turret;
    private final LauncherTuning tuning;
    
    // Cached shot parameters (updated each cycle)
    private ShotParameters currentShot;
    
    /**
     * Creates a ShootingCoordinator with the given subsystems and calculator.
     * 
     * @param launcher The flywheel launcher subsystem
     * @param turret The turret aiming subsystem
     * @param shotCalculator The shot calculator (with strategy configured)
     */
    public ShootingCoordinator(
        BangBangLauncher launcher, 
        SK26Turret turret,
        ShotCalculator shotCalculator
    ) {
        this.shotCalculator = shotCalculator;
        this.launcher = launcher;
        this.turret = turret;
        this.tuning = launcher.getLauncherTuning();
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
    
    /**
     * Get the most recently calculated shot parameters.
     * 
     * @return The current shot parameters, or null if not yet calculated
     */
    public ShotParameters getCurrentShot() {
        return currentShot;
    }
    
    /**
     * Check if the current shot is valid (within range, converged).
     * 
     * @return true if the shot is valid and can be taken
     */
    public boolean isValidShot() {
        return currentShot != null && currentShot.validShot() && currentShot.converged();
    }
    
    /**
     * Check if the launcher is at the target speed.
     * 
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
    
    // ==========================================================================
    //                              COMMANDS
    // ==========================================================================
    
    /**
     * Command that continuously aims and spins up, tracking a moving target.
     * 
     * <p>This is the MAIN command for "move and shoot" - it:
     * <ol>
     *   <li>Continuously recalculates shot parameters</li>
     *   <li>Aims the turret at the predicted target position</li>
     *   <li>Spins the flywheel to the required speed</li>
     *   <li>Accounts for robot motion during projectile flight</li>
     * </ol>
     * 
     * @param poseSupplier Supplier for current robot pose
     * @param velocitySupplier Supplier for current robot velocity
     * @param targetPosition Field-relative target position
     * @return Command that tracks and prepares to shoot
     */
    public Command aimAndSpinUpCommand(
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> velocitySupplier,
        Translation2d targetPosition
    ) {
        return Commands.run(() -> {
            // STEP 1: Update shot calculation with current robot state
            updateShotCalculation(
                poseSupplier.get(),
                velocitySupplier.get(),
                targetPosition
            );
            
            // STEP 2: Aim turret (if shot is valid)
            if (currentShot != null && currentShot.validShot()) {
                // Convert launcher yaw to turret angle
                // The shot calculation gives us field-relative angle,
                // we need to convert to turret-relative
                double turretAngleDeg = currentShot.launcherYaw().in(Degrees);
                turret.setAngleDegrees(turretAngleDeg);
            }
            
            // STEP 3: Spin flywheel
            // The BangBangLauncher will handle the control mode switching
            // (TorqueCurrentFOC for aggressive correction, VoltageOut for steady state)
            
        }, turret) // Require turret subsystem
        .alongWith(
            // Run flywheel in parallel
            launcher.runFixedSpeedCommand(() -> {
                if (currentShot != null && currentShot.validShot()) {
                    // Convert RPM to appropriate units
                    return RotationsPerSecond.of(
                        currentShot.flywheelSpeed().in(RPM) / 60.0
                    );
                }
                return RotationsPerSecond.zero();
            })
        );
    }
    
    /**
     * Command for a STATIONARY shot (robot not moving).
     * Simpler version that doesn't need velocity input.
     * 
     * @param poseSupplier Supplier for current robot pose
     * @param targetPosition Field-relative target position
     * @return Command that aims and spins up for a stationary shot
     */
    public Command stationaryShotCommand(
        Supplier<Pose2d> poseSupplier,
        Translation2d targetPosition
    ) {
        return aimAndSpinUpCommand(
            poseSupplier,
            () -> new ChassisSpeeds(),  // Zero velocity
            targetPosition
        );
    }
    
    /**
     * Full shooting sequence: aim, spin up, wait until ready, then shoot.
     * 
     * @param poseSupplier Supplier for robot pose
     * @param velocitySupplier Supplier for robot velocity
     * @param targetPosition Target to shoot at
     * @param feedCommand Command to feed the game piece into the launcher
     * @return Command that executes the full shooting sequence
     */
    public Command fullShootingSequence(
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> velocitySupplier,
        Translation2d targetPosition,
        Command feedCommand
    ) {
        return Commands.sequence(
            // Step 1: Aim and spin up until ready
            aimAndSpinUpCommand(poseSupplier, velocitySupplier, targetPosition)
                .until(this::isReadyToShoot),
            
            // Step 2: Feed the game piece while maintaining aim/speed
            aimAndSpinUpCommand(poseSupplier, velocitySupplier, targetPosition)
                .alongWith(feedCommand)
                .withTimeout(0.5),  // Feeding should be quick
            
            // Step 3: Stop the launcher
            launcher.stopCommand()
        );
    }
    
    // ==========================================================================
    //                              GETTERS
    // ==========================================================================
    
    /**
     * @return The shot calculator used by this coordinator
     */
    public ShotCalculator getShotCalculator() {
        return shotCalculator;
    }
    
    /**
     * @return The launcher subsystem
     */
    public BangBangLauncher getLauncher() {
        return launcher;
    }
    
    /**
     * @return The turret subsystem
     */
    public SK26Turret getTurret() {
        return turret;
    }
}
