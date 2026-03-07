package frc.robot.subsystems.fueldetection;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Represents a single piece of fuel that has been detected and projected
 * onto the field.  Tracks confidence/staleness so the fuel map can decide when
 * to keep or drop an entry.
 */
public class TrackedFuel {

    /** Unique ID assigned when the fuel is first added to the map. */
    public final int id;

    /** Current estimated field-space position */
    private Translation2d fieldPosition;

    /**
     * Confidence counter.  Incremented when the fuel is re-observed across
     * frames, decremented when it is not seen.  Removed at zero.
     */
    private int confidence;

    /** FPGA timestamp (seconds) of the last time this fuel was observed. */
    private double lastSeenTimestamp;

    /** Total number of frames this fuel has been observed. */
    private int totalObservations;

    // ==================== Constants ====================

    /** Max confidence value — higher = more persistent once established. */
    public static final int MAX_CONFIDENCE = 40; // ~0.8 s at 50 Hz

    /** Starting confidence when a fuel is first created. */
    public static final int INITIAL_CONFIDENCE = 8;

    /** Confidence gained per frame the fuel is re-observed. */
    public static final int CONFIDENCE_GAIN = 3;

    /** Confidence lost per frame the fuel is NOT observed. */
    public static final int CONFIDENCE_DECAY = 1;

    /**
     * Minimum confidence a fuel must reach before it is considered "confirmed"
     * and shown in the public fuel list.  This prevents brand-new, single-frame
     * ghost detections from flickering on screen.
     */
    public static final int DISPLAY_CONFIDENCE_THRESHOLD = 10;

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
     * Called when an incoming detection is matched to this tracked fuel
     * across frames.  Blends the new position into the existing estimate
     * and bumps confidence.
     *
     * <p>Uses an <b>adaptive alpha</b>: new / low-confidence fuels blend quickly
     * so they converge to the right spot, while well-established high-confidence
     * fuels barely move — dramatically reducing the visible drift when a ball
     * is stationary.
     *
     * @param newPosition  New field-space position from the latest detection
     * @param timestamp    FPGA timestamp of this observation
     * @param baseAlpha    Base EMA blend weight (the map-level tuning value).
     *                     The actual alpha applied is scaled down by confidence.
     */
    public void update(Translation2d newPosition, double timestamp, double baseAlpha) {
        // Scale alpha inversely with confidence:
        //   conf = INITIAL  → factor ≈ 1.0  (snap quickly to new data)
        //   conf = MAX      → factor ≈ 0.20 (almost locked in place)
        double confRatio = (double) confidence / MAX_CONFIDENCE; // 0..1
        double adaptiveAlpha = baseAlpha * (1.0 - 0.80 * confRatio);

        fieldPosition = fieldPosition.interpolate(newPosition, adaptiveAlpha);
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

    /**
     * Returns true when this fuel has been seen enough times to be considered
     * a real ball rather than a single-frame ghost.  Use this to filter the
     * display list and prevent flicker.
     */
    public boolean isConfirmed() {
        return confidence >= DISPLAY_CONFIDENCE_THRESHOLD;
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
