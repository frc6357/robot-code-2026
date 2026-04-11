package frc.robot.subsystems.fueldetection;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Maintains a persistent, flicker-resistant map of fuel (balls) on the field.
 *
 * Design for a 400-ball field
 *
 * Cross-frame matching: When the same ball appears in consecutive
 *       frames it is matched to its existing {@link TrackedFuel} entry (within
 *       {@link #MATCH_RADIUS_METERS}) rather than creating a duplicate.  This
 *       keeps the map from exploding with redundant entries.
 * No same-frame merging: Two detections in the <em>same</em>
 *       frame are never combined — the neural net already distinguishes
 *       individual balls even when they are close together.
 * Confidence / decay: Matched fuels gain confidence; unmatched
 *       fuels lose it.  Once confidence hits zero the entry is evicted.
 * Capacity management: A hard cap ({@link #MAX_TRACKED}) prevents
 *       unbounded growth.  When the cap is reached, the lowest-confidence
 *       entry is evicted to make room for a new detection.
 */
public class FuelMap {

    // ==================== Tuning constants ====================

    /**
     * If a new detection is within this distance of an existing
     * tracked fuel, it is considered the same physical ball (cross-frame
     * match).  This is intentionally generous to account for pose-estimation
     * jitter between frames.
     * Ball diameter ≈ 0.15 m; a 0.40 m radius handles typical noise.
     */
    public static final double MATCH_RADIUS_METERS = 0.40;

    /**
     * EMA blend weight when updating a matched fuel's position.
     * Lower = smoother / slower to move, higher = snappier.
     * Note: TrackedFuel further scales this down adaptively based on
     * confidence, so high-confidence balls will barely drift.
     */
    public static final double POSITION_ALPHA = 0.30;

    /**
     * Hard cap on tracked fuels.  The Limelight NN can see ~5-10 balls at
     * once, but as the robot drives around a 400-ball field the map
     * accumulates entries.  This cap keeps memory and iteration costs bounded.
     */
    public static final int MAX_TRACKED = 50;

    // ==================== State ====================

    private final List<TrackedFuel> trackedFuels = new ArrayList<>();
    private int nextId = 0;

    // ==================== Public API ====================

    /**
     * Feed this cycle's detections into the map.  Call once per periodic().
     *
     * Algorithm (per frame):
     * 
     *   For each new detection, find the closest existing fuel
     *       within {@link #MATCH_RADIUS_METERS}.  Each existing fuel may only
     *       be claimed by one detection (greedy nearest-first).
     *   Matched fuels get their position blended and confidence bumped.
     *   Unmatched detections become new entries (evicting the weakest
     *       entry if at capacity).
     *   Existing fuels that were NOT matched this frame decay.
     *
     * @param fieldPositions Array of field-space positions for every ball
     *                       detected this frame.
     */
    public void update(Translation2d[] fieldPositions) {
        double now = Timer.getFPGATimestamp();

        int existingCount = trackedFuels.size();
        boolean[] matched = new boolean[existingCount];

        // ---- Match each detection to an existing fuel ----
        for (Translation2d newPos : fieldPositions) {
            // Reject positions that are clearly off the field
            if (newPos.getX() < -0.5 || newPos.getX() > 17.0
                    || newPos.getY() < -0.5 || newPos.getY() > 9.0) {
                continue;
            }

            // Find closest UNMATCHED existing fuel within the match radius
            int bestIdx = -1;
            double bestDist = MATCH_RADIUS_METERS;

            for (int i = 0; i < existingCount; i++) {
                if (matched[i]) continue; // already claimed this frame
                double dist = trackedFuels.get(i).getFieldPosition().getDistance(newPos);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestIdx = i;
                }
            }

            if (bestIdx >= 0) {
                // Same ball seen again — blend position & bump confidence
                trackedFuels.get(bestIdx).update(newPos, now, POSITION_ALPHA);
                matched[bestIdx] = true;
            } else {
                // Genuinely new ball, add it
                if (trackedFuels.size() >= MAX_TRACKED) {
                    evictWeakest();
                }
                trackedFuels.add(new TrackedFuel(nextId++, newPos, now));
            }
        }

        // ---- Decay unmatched existing fuels & remove expired ----
        for (int i = existingCount - 1; i >= 0; i--) {
            if (!matched[i]) {
                trackedFuels.get(i).decay();
                if (trackedFuels.get(i).isExpired()) {
                    trackedFuels.remove(i);
                }
            }
        }
    }

    // ==================== Internals ====================

    /**
     * Removes the tracked fuel with the lowest confidence to make room for a
     * new entry.  Ties are broken by oldest timestamp (least recently seen).
     */
    private void evictWeakest() {
        if (trackedFuels.isEmpty()) return;

        int worstIdx = 0;
        int worstConf = trackedFuels.get(0).getConfidence();
        double worstTime = trackedFuels.get(0).getLastSeenTimestamp();

        for (int i = 1; i < trackedFuels.size(); i++) {
            TrackedFuel f = trackedFuels.get(i);
            int conf = f.getConfidence();
            double time = f.getLastSeenTimestamp();

            if (conf < worstConf || (conf == worstConf && time < worstTime)) {
                worstIdx = i;
                worstConf = conf;
                worstTime = time;
            }
        }

        trackedFuels.remove(worstIdx);
    }

    /** Returns an unmodifiable snapshot of all currently tracked fuels. */
    public List<TrackedFuel> getTrackedFuels() {
        return List.copyOf(trackedFuels);
    }

    /**
     * Returns only fuels whose confidence exceeds
     * {@link TrackedFuel#DISPLAY_CONFIDENCE_THRESHOLD}.  Use this for display
     * and decision-making to avoid flickering ghost entries.
     */
    public List<TrackedFuel> getConfirmedFuels() {
        List<TrackedFuel> confirmed = new ArrayList<>();
        for (TrackedFuel f : trackedFuels) {
            if (f.isConfirmed()) {
                confirmed.add(f);
            }
        }
        return confirmed;
    }

    /** Number of fuels currently being tracked. */
    public int size() {
        return trackedFuels.size();
    }

    /** Number of confirmed (display-worthy) fuels. */
    public int confirmedSize() {
        int count = 0;
        for (TrackedFuel f : trackedFuels) {
            if (f.isConfirmed()) count++;
        }
        return count;
    }

    /** Removes all tracked fuels (e.g. on mode change). */
    public void clear() {
        trackedFuels.clear();
    }

    /**
     * Removes every tracked fuel within {@code radius} metres of the given
     * position.  Used in simulation to "collect" balls when the robot drives
     * over them.
     *
     * @return the number of fuels removed.
     */
    public int removeNear(Translation2d position, double radius) {
        int removed = 0;
        for (int i = trackedFuels.size() - 1; i >= 0; i--) {
            if (trackedFuels.get(i).getFieldPosition().getDistance(position) <= radius) {
                trackedFuels.remove(i);
                removed++;
            }
        }
        return removed;
    }
}
