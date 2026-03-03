package frc.robot.subsystems.launcher.moveandshoot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.units.measure.Distance;

/**
 * Sealed interface for shot calculation strategies.
 *
 * <p>Defines the contract for calculating shooting parameters given:
 * <ul>
 *   <li>Current 3D shooter pose</li>
 *   <li>Field-relative shooter velocity</li>
 *   <li>3D target position</li>
 *   <li>Environmental/runtime context</li>
 * </ul>
 *
 * <p>Strategies are sealed to:
 * <ul>
 *   <li>InterpolatedShotStrategy - Mechanical Advantage style iterative lookahead</li>
 *   <li>BallisticShotStrategy - Physics-based projectile motion (future)</li>
 * </ul>
 *
 * <p>Implementations must be thread-safe for concurrent calculations.
 */
public sealed interface ShotCalculationStrategy
    permits InterpolatedShotStrategy, BallisticShotStrategy {

    /**
     * Calculate shot parameters for the given shooter state and target.
     *
     * @param shooterPose Current shooter pose in 3D space (field-relative)
     * @param shooterVelocity Field-relative velocity as Twist3d
     * @param target Target position in 3D space (field-relative)
     * @param context Additional calculation context (environmental factors, state)
     * @return Calculated shot parameters with typed units
     */
    ShotParameters calculate(
        Pose3d shooterPose,
        Twist3d shooterVelocity,
        Translation3d target,
        CalculationContext context
    );

    /**
     * Get valid shooting range for this strategy.
     *
     * @return Range with min/max effective distances
     */
    Range getValidRange();

    /**
     * Strategy name for telemetry and logging.
     *
     * @return Human-readable strategy name
     */
    String getName();

    /**
     * Valid shooting range with minimum and maximum effective distances.
     */
    record Range(Distance min, Distance max) {
        /**
         * Checks if a distance is within valid range.
         *
         * @param distance Distance to check
         * @return true if distance is within [min, max]
         */
        public boolean isValid(Distance distance) {
            double meters = distance.in(Meters);
            double minMeters = min.in(Meters);
            double maxMeters = max.in(Meters);
            return meters >= minMeters && meters <= maxMeters;
        }
    }
}
