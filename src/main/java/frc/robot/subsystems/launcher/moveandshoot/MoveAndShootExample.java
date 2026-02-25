package frc.robot.subsystems.launcher.moveandshoot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.launcher.mechanisms.BangBangLauncher;
import frc.robot.subsystems.launcher.moveandshoot.ShotCalculationStrategy.Range;
import frc.robot.subsystems.turret.SK26Turret;

/**
 * =============================================================================
 *                    MOVE-AND-SHOOT COMPLETE EXAMPLE
 * =============================================================================
 * 
 * This file demonstrates the COMPLETE flow from requesting a shot to spinning
 * the flywheel, using the move-and-shoot calculation system with BangBangLauncher.
 * 
 * ARCHITECTURE OVERVIEW:
 * ┌─────────────────────────────────────────────────────────────────────────────┐
 * │                                                                             │
 * │  ┌──────────────┐    ┌───────────────┐    ┌─────────────────────────────┐   │
 * │  │   Command    │───▶│ ShotCalculator│───▶│ InterpolatedShotStrategy   │   │
 * │  │ (requests    │    │ (coordinates) │    │ (calculates parameters)     │   │
 * │  │  a shot)     │    └───────────────┘    └─────────────────────────────┘   │
 * │  └──────────────┘            │                                              │
 * │                              ▼                                              │
 * │                    ┌───────────────────┐                                    │
 * │                    │  ShotParameters   │                                    │
 * │                    │ (flywheel RPM,    │                                    │
 * │                    │  turret angle,    │                                    │
 * │                    │  time of flight)  │                                    │
 * │                    └─────────┬─────────┘                                    │
 * │                              │                                              │
 * │              ┌───────────────┼───────────────┐                              │
 * │              ▼               ▼               ▼                              │
 * │     ┌──────────────┐ ┌──────────────┐ ┌──────────────┐                      │
 * │     │BangBangLaunch│ │   SK26Turret │ │LauncherTuning│                      │
 * │     │(spins wheel) │ │(aims turret) │ │ (telemetry)  │                      │
 * │     └──────────────┘ └──────────────┘ └──────────────┘                      │
 * │                                                                             │
 * └─────────────────────────────────────────────────────────────────────────────┘
 * 
 * =============================================================================
 */
public class MoveAndShootExample {

    // ==========================================================================
    //                         STEP 1: BUILD THE STRATEGY
    // ==========================================================================
    
    /**
     * Creates the InterpolatedShotStrategy with characterization data.
     * 
     * The strategy uses lookup tables (InterpolatingDoubleTreeMap) that map:
     *   - Distance (meters) → Flywheel RPM
     *   - Distance (meters) → Time of Flight (seconds)
     * 
     * These values come from CHARACTERIZATION - shooting at known distances
     * and recording what RPM was needed and how long the ball took to arrive.
     */
    public static InterpolatedShotStrategy createShotStrategy() {
        
        // ===== FLYWHEEL SPEED MAP =====
        // Key: distance in meters
        // Value: flywheel RPM needed to reach that distance
        InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
        flywheelSpeedMap.put(1.0, 2000.0);   // 1m → 2000 RPM
        flywheelSpeedMap.put(2.0, 2800.0);   // 2m → 2800 RPM
        flywheelSpeedMap.put(3.0, 3400.0);   // 3m → 3400 RPM
        flywheelSpeedMap.put(4.0, 3900.0);   // 4m → 3900 RPM
        flywheelSpeedMap.put(5.0, 4300.0);   // 5m → 4300 RPM
        // WPILib will INTERPOLATE between these points automatically!
        // So 2.5m → ~3100 RPM (interpolated between 2800 and 3400)

        // ===== TIME OF FLIGHT MAP =====
        // Key: distance in meters
        // Value: time in seconds for projectile to reach target
        InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
        timeOfFlightMap.put(1.0, 0.3);    // 1m → 0.3 seconds
        timeOfFlightMap.put(2.0, 0.45);   // 2m → 0.45 seconds
        timeOfFlightMap.put(3.0, 0.6);    // 3m → 0.6 seconds
        timeOfFlightMap.put(4.0, 0.75);   // 4m → 0.75 seconds
        timeOfFlightMap.put(5.0, 0.9);    // 5m → 0.9 seconds

        // ===== BUILD THE STRATEGY =====
        return new InterpolatedShotStrategy.Builder()
            .withFlywheelSpeedMap(flywheelSpeedMap)
            .withTimeOfFlightMap(timeOfFlightMap)
            .withFixedLaunchAngle(Degrees.of(55))           // Fixed 55° launch angle
            .withMaxIterations(10)                           // Max lookahead iterations
            .withConvergenceThreshold(Centimeters.of(5))    // Stop when distance converges within 5cm
            .withValidRange(new Range(Meters.of(1.0), Meters.of(5.0)))  // Valid shooting range
            .build();
    }

    // ==========================================================================
    //                       STEP 2: BUILD THE CALCULATOR
    // ==========================================================================
    
    /**
     * Creates the ShotCalculator that coordinates everything.
     * 
     * The calculator handles:
     *   1. Phase delay compensation (predict future robot position)
     *   2. Robot-to-shooter transformation (where is the shooter on the robot?)
     *   3. Delegating to the strategy for actual calculation
     */
    public static ShotCalculator createShotCalculator() {
        
        // Where is the shooter relative to the robot center?
        // This Transform3d describes the position and orientation offset
        Transform3d robotToShooter = new Transform3d(
            Meters.of(0.0),     // X: 0m forward from robot center
            Meters.of(0.0),     // Y: 0m left from robot center  
            Meters.of(0.5),     // Z: 0.5m up from ground (shooter height)
            new edu.wpi.first.math.geometry.Rotation3d()  // No rotation offset
        );

        return new ShotCalculator.Builder()
            .withStrategy(createShotStrategy())
            .withRobotToShooterTransform(robotToShooter)
            .withPhaseDelay(0.02)  // 20ms processing delay compensation
            .build();
    }

    // ==========================================================================
    //                    STEP 3: THE SHOOTING SUBSYSTEM
    // ==========================================================================
    
    /**
     * A coordinating class that ties everything together.
     * In practice, this might be part of a larger "Superstructure" class.
     */
    public static class ShootingCoordinator {
        
        private final ShotCalculator shotCalculator;
        private final BangBangLauncher launcher;
        private final SK26Turret turret;
        private final LauncherTuning tuning;
        
        // Cached shot parameters (updated each cycle)
        private ShotParameters currentShot;
        
        public ShootingCoordinator(BangBangLauncher launcher, SK26Turret turret) {
            this.shotCalculator = createShotCalculator();
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
         */
        public ShotParameters getCurrentShot() {
            return currentShot;
        }
        
        /**
         * Check if the current shot is valid (within range, converged).
         */
        public boolean isValidShot() {
            return currentShot != null && currentShot.validShot() && currentShot.converged();
        }
        
        /**
         * Check if the launcher is at the target speed.
         */
        public boolean isLauncherReady() {
            return launcher.isAtGoal();
        }
        
        /**
         * Check if the turret is aimed at the target.
         */
        public boolean isTurretReady() {
            return turret.atTarget();
        }
        
        /**
         * Check if everything is ready to shoot.
         */
        public boolean isReadyToShoot() {
            return isValidShot() && isLauncherReady() && isTurretReady();
        }
        
        // ======================================================================
        //                        COMMANDS
        // ======================================================================
        
        /**
         * Command that continuously aims and spins up, tracking a moving target.
         * 
         * This is the MAIN command you'd use for "move and shoot" - it:
         *   1. Continuously recalculates shot parameters
         *   2. Aims the turret at the predicted target position
         *   3. Spins the flywheel to the required speed
         *   4. Accounts for robot motion during projectile flight
         */
        public Command aimAndSpinUpCommand(
            java.util.function.Supplier<Pose2d> poseSupplier,
            java.util.function.Supplier<ChassisSpeeds> velocitySupplier,
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
         */
        public Command stationaryShotCommand(
            java.util.function.Supplier<Pose2d> poseSupplier,
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
         */
        public Command fullShootingSequence(
            java.util.function.Supplier<Pose2d> poseSupplier,
            java.util.function.Supplier<ChassisSpeeds> velocitySupplier,
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
    }

    // ==========================================================================
    //                    STEP 4: USAGE EXAMPLE
    // ==========================================================================
    
    /**
     * Example of how you'd use this in RobotContainer or a command factory.
     */
    public static void usageExample() {
        // Assume these are created elsewhere (RobotContainer)
        BangBangLauncher launcher = new BangBangLauncher();
        SK26Turret turret = null; // = new SK26Turret();
        
        // Create the coordinator
        ShootingCoordinator coordinator = new ShootingCoordinator(launcher, turret);
        
        // Define the target (e.g., speaker position)
        Translation2d speakerPosition = new Translation2d(0.0, 5.5);  // Field coordinates
        
        // Create a command that tracks and shoots at the speaker while moving
        Command moveAndShootCommand = coordinator.aimAndSpinUpCommand(
            () -> new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(45)),  // Robot pose supplier
            () -> new ChassisSpeeds(1.0, 0.5, 0.1),                   // Robot velocity supplier
            speakerPosition
        );
        
        // Or create a full shooting sequence
        Command fullSequence = coordinator.fullShootingSequence(
            () -> new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(45)),
            () -> new ChassisSpeeds(1.0, 0.5, 0.1),
            speakerPosition,
            Commands.none()  // Replace with actual feed command
        );
    }

    // ==========================================================================
    //                    DATA FLOW DIAGRAM
    // ==========================================================================
    /*
     * Here's exactly what happens when you request a shot:
     * 
     * 1. COMMAND CALLS updateShotCalculation()
     *    │
     *    ├─ Input: Robot pose (2.0, 3.0, 45°)
     *    ├─ Input: Robot velocity (vx=1.0, vy=0.5, ω=0.1)
     *    └─ Input: Target position (0.0, 5.5)
     * 
     * 2. SHOT CALCULATOR PROCESSES
     *    │
     *    ├─ Applies robot-to-shooter transform
     *    │   └─ Shooter pose = robot pose + (0, 0, 0.5m up)
     *    │
     *    ├─ Converts velocity to Twist3d
     *    │   └─ (vx=1.0, vy=0.5, vz=0, rx=0, ry=0, rz=0.1)
     *    │
     *    └─ Delegates to InterpolatedShotStrategy
     * 
     * 3. INTERPOLATED STRATEGY CALCULATES
     *    │
     *    ├─ Initial distance: sqrt((2-0)² + (3-5.5)²) = 3.2m
     *    │
     *    ├─ ITERATIVE LOOKAHEAD LOOP:
     *    │   │
     *    │   ├─ Iteration 1:
     *    │   │   ├─ ToF for 3.2m → ~0.58s (interpolated)
     *    │   │   ├─ Future position: (2 + 1.0*0.58, 3 + 0.5*0.58) = (2.58, 3.29)
     *    │   │   └─ New distance: 3.05m
     *    │   │
     *    │   ├─ Iteration 2:
     *    │   │   ├─ ToF for 3.05m → ~0.55s
     *    │   │   ├─ Future position: (2.55, 3.275)
     *    │   │   └─ New distance: 3.08m (converged within 5cm!)
     *    │   │
     *    │   └─ Converged after 2 iterations
     *    │
     *    ├─ Flywheel RPM: 3.08m → ~3420 RPM (interpolated)
     *    │
     *    ├─ Launcher yaw: atan2(5.5-3.275, 0-2.55) = -65.3°
     *    │
     *    └─ Yaw velocity feedforward: ~0.36 rad/s
     * 
     * 4. SHOT PARAMETERS RETURNED
     *    │
     *    ├─ flywheelSpeed: 3420 RPM
     *    ├─ launchAngle: 55° (fixed)
     *    ├─ launcherYaw: -65.3°
     *    ├─ launcherYawVelocity: 0.36 rad/s
     *    ├─ effectiveDistance: 3.08m
     *    ├─ timeOfFlight: 0.55s
     *    ├─ converged: true
     *    └─ validShot: true (3.08m is within 1-5m range)
     * 
     * 5. SUBSYSTEMS COMMANDED
     *    │
     *    ├─ TURRET: setAngleDegrees(-65.3°)
     *    │   └─ Turret rotates to aim at predicted intercept point
     *    │
     *    └─ BANG BANG LAUNCHER: runVelocity(57 RPS)  // 3420 RPM / 60
     *        │
     *        ├─ If far from target: TORQUE_CURRENT_BANG_BANG mode
     *        │   └─ Aggressive 54A torque current for fast spinup
     *        │
     *        └─ If close to target: VOLTAGE_BANG_BANG mode
     *            └─ Feedforward voltage to maintain speed
     * 
     * 6. TELEMETRY PUBLISHED
     *    │
     *    └─ LauncherTuning publishes to NetworkTables:
     *        ├─ BBLauncher/Telemetry/FlywheelRPM: 3420
     *        ├─ BBLauncher/Telemetry/LauncherYaw: -65.3
     *        ├─ BBLauncher/Telemetry/Distance: 3.08
     *        ├─ BBLauncher/Telemetry/TimeOfFlight: 0.55
     *        ├─ BBLauncher/Telemetry/Converged: true
     *        └─ BBLauncher/Telemetry/ValidShot: true
     */
}
