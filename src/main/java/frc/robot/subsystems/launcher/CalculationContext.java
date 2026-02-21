package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/**
 * Context for shot calculations containing timestamp and optional metadata.
 *
 * <p>Simple version - just timestamp for now. Can extend later with:
 * <ul>
 *   <li>Environmental factors (temperature, wind)</li>
 *   <li>Game state (alliance, match time)</li>
 * </ul>
 */
public record CalculationContext(
    double timestamp,
    Optional<Alliance> alliance
) {
    /**
     * Create context with just a timestamp.
     */
    public static CalculationContext create(double timestamp) {
        return new CalculationContext(timestamp, Optional.empty());
    }

    /**
     * Create context with timestamp and alliance.
     */
    public static CalculationContext create(double timestamp, Alliance alliance) {
        return new CalculationContext(timestamp, Optional.of(alliance));
    }
}
