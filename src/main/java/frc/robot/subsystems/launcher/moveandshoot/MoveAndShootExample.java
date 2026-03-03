// package frc.robot.subsystems.launcher.moveandshoot;

// import static edu.wpi.first.units.Units.*;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.Konstants.LauncherConstants;
// import frc.robot.subsystems.launcher.mechanisms.BangBangLauncher;
// import frc.robot.subsystems.launcher.moveandshoot.ShotCalculationStrategy.Range;
// import frc.robot.subsystems.turret.SK26Turret;

// /**
//  * =============================================================================
//  *                    MOVE-AND-SHOOT COMPLETE EXAMPLE
//  * =============================================================================
//  * 
//  * This file demonstrates the COMPLETE flow from requesting a shot to spinning
//  * the flywheel, using the move-and-shoot calculation system with BangBangLauncher.
//  * 
//  * ARCHITECTURE OVERVIEW:
//  * ┌─────────────────────────────────────────────────────────────────────────────┐
//  * │                                                                             │
//  * │  ┌──────────────┐    ┌───────────────┐    ┌─────────────────────────────┐   │
//  * │  │   Command    │───▶│ ShotCalculator│───▶│ InterpolatedShotStrategy   │   │
//  * │  │ (requests    │    │ (coordinates) │    │ (calculates parameters)     │   │
//  * │  │  a shot)     │    └───────────────┘    └─────────────────────────────┘   │
//  * │  └──────────────┘            │                                              │
//  * │                              ▼                                              │
//  * │                    ┌───────────────────┐                                    │
//  * │                    │  ShotParameters   │                                    │
//  * │                    │ (flywheel RPM,    │                                    │
//  * │                    │  turret angle,    │                                    │
//  * │                    │  time of flight)  │                                    │
//  * │                    └─────────┬─────────┘                                    │
//  * │                              │                                              │
//  * │              ┌───────────────┼───────────────┐                              │
//  * │              ▼               ▼               ▼                              │
//  * │     ┌──────────────┐ ┌──────────────┐ ┌──────────────┐                      │
//  * │     │BangBangLaunch│ │   SK26Turret │ │LauncherTuning│                      │
//  * │     │(spins wheel) │ │(aims turret) │ │ (telemetry)  │                      │
//  * │     └──────────────┘ └──────────────┘ └──────────────┘                      │
//  * │                                                                             │
//  * └─────────────────────────────────────────────────────────────────────────────┘
//  * 
//  * =============================================================================
//  */
// public class MoveAndShootExample {
//     // ==========================================================================
//     //                    STEP 3: USAGE EXAMPLE
//     // ==========================================================================
    
//     /**
//      * Example of how you'd use this in RobotContainer or a command factory.
//      * 
//      * The ShootingCoordinator class (now in its own file) ties together:
//      *   - ShotCalculator (calculates shot parameters)
//      *   - BangBangLauncher (spins flywheel)
//      *   - SK26Turret (aims at target)
//      *   - LauncherTuning (telemetry)
//      */
//     @SuppressWarnings("unused")
//     public static void usageExample() {
//         // Assume these are created elsewhere (RobotContainer)
//         BangBangLauncher launcher = new BangBangLauncher();
//         SK26Turret turret = null; // = new SK26Turret();
        
//         // Create the shot calculator with strategy
//         ShotCalculator shotCalculator = createShotCalculator();
        
//         // Create the coordinator with all components
//         ShootingCoordinator coordinator = new ShootingCoordinator(
//             launcher, 
//             turret,
//             shotCalculator
//         );
    
        
//         // Create a command that tracks and shoots at the speaker while moving
//         Command moveAndShootCommand = coordinator.aimAndSpinUpCommand(
//             () -> new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(45)),  // Robot pose supplier
//             () -> new ChassisSpeeds(1.0, 0.5, 0.1),                   // Robot velocity supplier
//             speakerPosition
//         );
        
//         // Or create a full shooting sequence
//         Command fullSequence = coordinator.fullShootingSequence(
//             () -> new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(45)),
//             () -> new ChassisSpeeds(1.0, 0.5, 0.1),
//             speakerPosition,
//             Commands.none()  // Replace with actual feed command
//         );
//     }

//     // ==========================================================================
//     //                    DATA FLOW DIAGRAM
//     // ==========================================================================
//     /*
//      * Here's exactly what happens when you request a shot:
//      * 
//      * 1. COMMAND CALLS updateShotCalculation()
//      *    │
//      *    ├─ Input: Robot pose (2.0, 3.0, 45°)
//      *    ├─ Input: Robot velocity (vx=1.0, vy=0.5, ω=0.1)
//      *    └─ Input: Target position (0.0, 5.5)
//      * 
//      * 2. SHOT CALCULATOR PROCESSES
//      *    │
//      *    ├─ Applies robot-to-shooter transform
//      *    │   └─ Shooter pose = robot pose + (0, 0, 0.5m up)
//      *    │
//      *    ├─ Converts velocity to Twist3d
//      *    │   └─ (vx=1.0, vy=0.5, vz=0, rx=0, ry=0, rz=0.1)
//      *    │
//      *    └─ Delegates to InterpolatedShotStrategy
//      * 
//      * 3. INTERPOLATED STRATEGY CALCULATES
//      *    │
//      *    ├─ Initial distance: sqrt((2-0)² + (3-5.5)²) = 3.2m
//      *    │
//      *    ├─ ITERATIVE LOOKAHEAD LOOP:
//      *    │   │
//      *    │   ├─ Iteration 1:
//      *    │   │   ├─ ToF for 3.2m → ~0.58s (interpolated)
//      *    │   │   ├─ Future position: (2 + 1.0*0.58, 3 + 0.5*0.58) = (2.58, 3.29)
//      *    │   │   └─ New distance: 3.05m
//      *    │   │
//      *    │   ├─ Iteration 2:
//      *    │   │   ├─ ToF for 3.05m → ~0.55s
//      *    │   │   ├─ Future position: (2.55, 3.275)
//      *    │   │   └─ New distance: 3.08m (converged within 5cm!)
//      *    │   │
//      *    │   └─ Converged after 2 iterations
//      *    │
//      *    ├─ Flywheel RPM: 3.08m → ~3420 RPM (interpolated)
//      *    │
//      *    ├─ Launcher yaw: atan2(5.5-3.275, 0-2.55) = -65.3°
//      *    │
//      *    └─ Yaw velocity feedforward: ~0.36 rad/s
//      * 
//      * 4. SHOT PARAMETERS RETURNED
//      *    │
//      *    ├─ flywheelSpeed: 3420 RPM
//      *    ├─ launchAngle: 55° (fixed)
//      *    ├─ launcherYaw: -65.3°
//      *    ├─ launcherYawVelocity: 0.36 rad/s
//      *    ├─ effectiveDistance: 3.08m
//      *    ├─ timeOfFlight: 0.55s
//      *    ├─ converged: true
//      *    └─ validShot: true (3.08m is within 1-5m range)
//      * 
//      * 5. SUBSYSTEMS COMMANDED
//      *    │
//      *    ├─ TURRET: setAngleDegrees(-65.3°)
//      *    │   └─ Turret rotates to aim at predicted intercept point
//      *    │
//      *    └─ BANG BANG LAUNCHER: runVelocity(57 RPS)  // 3420 RPM / 60
//      *        │
//      *        ├─ If far from target: TORQUE_CURRENT_BANG_BANG mode
//      *        │   └─ Aggressive 54A torque current for fast spinup
//      *        │
//      *        └─ If close to target: VOLTAGE_BANG_BANG mode
//      *            └─ Feedforward voltage to maintain speed
//      * 
//      * 6. TELEMETRY PUBLISHED
//      *    │
//      *    └─ LauncherTuning publishes to NetworkTables:
//      *        ├─ BBLauncher/Telemetry/FlywheelRPM: 3420
//      *        ├─ BBLauncher/Telemetry/LauncherYaw: -65.3
//      *        ├─ BBLauncher/Telemetry/Distance: 3.08
//      *        ├─ BBLauncher/Telemetry/TimeOfFlight: 0.55
//      *        ├─ BBLauncher/Telemetry/Converged: true
//      *        └─ BBLauncher/Telemetry/ValidShot: true
//      */
// }
