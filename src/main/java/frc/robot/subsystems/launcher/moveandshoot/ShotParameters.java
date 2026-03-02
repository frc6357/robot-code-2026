package frc.robot.subsystems.launcher.moveandshoot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.*;

/**
 * Record containing all calculated shooting parameters.
 *
 * <p>Includes:
 * <ul>
 *   <li>Core parameters: flywheel speed, launch angle, launcher yaw</li>
 *   <li>Velocity feedforward for launcher yaw tracking</li>
 *   <li>Effective target position and distance after motion compensation</li>
 *   <li>Time of flight for the shot</li>
 *   <li>Validity flags and debug metadata</li>
 * </ul>
 *
 * <p>All physical quantities use WPILib Measure types for type safety.
 */
public record ShotParameters(
    // Core shooting parameters
    AngularVelocity flywheelSpeed,
    Angle launchAngle,
    Angle launcherYaw,
    AngularVelocity launcherYawVelocity,

    Translation3d initialTarget,
    // Target information (after motion compensation)
    Translation3d effectiveTarget,
    Distance effectiveDistance,
    Time timeOfFlight,

    // Debug and metadata
    String calculationStrategy,
    int iterationsUsed,
    boolean converged,
    boolean validShot,
    double timestamp
) {

}
