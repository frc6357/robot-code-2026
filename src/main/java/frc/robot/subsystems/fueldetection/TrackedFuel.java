package frc.robot.subsystems.fueldetection;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Represents a single piece of fuel (ball) that has been detected and projected
 * onto the field.  Tracks confidence/staleness so the fuel map can decide when
 * to keep or drop an entry.
 */
public class TrackedFuel {

    /** Unique ID assigned when the fuel is first added to the map. */
    public final int id;

    /** Current estimated field-space position (meters, WPILib blue-origin). */
    private Translation2d fieldPosition;

    /**
     * Confidence counter.  Incremented every cycle the fuel is re-observed,
     * decremented every cycle it is <b>not</b> seen.  The fuel is removed
     * from the map when this drops to zero.
     */
    private int confidence;

    /** FPGA timestamp (seconds) of the last time this fuel was observed. */
    private double lastSeenTimestamp;

    /** How many total frames this fuel has ever been observed. */
    private int totalObservations;

    // ==================== Constants ====================

    /** Max confidence value (caps how long a fuel persists after disappearing). */
    public static final int MAX_CONFIDENCE = 30; // ~0.6 s at 50 Hz

    /** Starting confidence when a fuel is first created. */
    public static final int INITIAL_CONFIDENCE = 5;

    /** Confidence gained per frame the fuel is re-observed. */
    public static final int CONFIDENCE_GAIN = 3;

    /** Confidence lost per frame the fuel is NOT observed. */
    public static final int CONFIDENCE_DECAY = 1;

    // ==================== Constructor ====================

    public TrackedFuel(int id, Translation2d fieldPosition, double timestamp) {
        this.id = id;
        this.fieldPosition = fieldPosition;
        this.confidence = INITIAL_CONFIDENCE;
        this.lastSeenTimestamp = timestamp;
        this.totalObservations = 1;
    }

    // ==================== Update ====================

    /**
     * Called when an incoming detection is matched to this tracked fuel.
     * Blends the new position into the existing estimate with a simple
     * exponential-moving-average and bumps confidence.
     *
     * @param newPosition  New field-space position from the latest detection
     * @param timestamp    FPGA timestamp of this observation
     * @param alpha        Blend weight for the new measurement (0 = ignore, 1 = snap)
     */
    public void update(Translation2d newPosition, double timestamp, double alpha) {
        // EMA blend: pos = (1-α)·old + α·new
        fieldPosition = fieldPosition.interpolate(newPosition, alpha);
        confidence = Math.min(confidence + CONFIDENCE_GAIN, MAX_CONFIDENCE);
        lastSeenTimestamp = timestamp;
        totalObservations++;
    }

    /**
     * Called once per cycle for every tracked fuel that was NOT observed this
     * frame.  Decays confidence; caller should remove the fuel when
     * {@link #isExpired()} returns true.
     */
    public void decay() {
        confidence -= CONFIDENCE_DECAY;
    }

    // ==================== Queries ====================

    public boolean isExpired() {
        return confidence <= 0;
    }

    public Translation2d getFieldPosition() {
        return fieldPosition;
    }

    public int getConfidence() {
        return confidence;
    }

    public double getLastSeenTimestamp() {
        return lastSeenTimestamp;
    }

    public int getTotalObservations() {
        return totalObservations;
    }
}
