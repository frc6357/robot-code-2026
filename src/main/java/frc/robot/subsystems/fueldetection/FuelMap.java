package frc.robot.subsystems.fueldetection;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Maintains a persistent, flicker-resistant map of fuel (balls) on the field.
 *
 * <h2>How it works</h2>
 * <ol>
 *   <li>Each cycle, new field-space detections are fed in via {@link #update}.</li>
 *   <li>Each new detection is matched to the nearest existing tracked fuel
 *       within {@link #MERGE_RADIUS_METERS}.  If a match is found the
 *       position is blended (EMA) and confidence is bumped.</li>
 *   <li>Unmatched detections create new tracked fuel entries.</li>
 *   <li>Tracked fuels that were <b>not</b> observed this cycle have their
 *       confidence decayed.  Once confidence reaches zero the entry is removed.</li>
 * </ol>
 *
 * This approach prevents:
 * <ul>
 *   <li><b>Flickering:</b> a fuel that drops out for a few frames stays on the
 *       map because its confidence drains slowly.</li>
 *   <li><b>Duplicates:</b> detections close to an existing fuel are merged
 *       rather than creating a new entry.</li>
 * </ul>
 */
public class FuelMap {

    // ==================== Tuning constants ====================

    /**
     * If a new detection is within this distance (meters) of an existing
     * tracked fuel, they are considered the same ball.
     * Ball diameter is 5.91 in ≈ 0.15 m; use a generous merge radius.
     */
    public static final double MERGE_RADIUS_METERS = 0.50;

    /**
     * EMA blend weight when updating a tracked fuel's position.
     * Lower = smoother / slower to move, higher = snappier.
     */
    public static final double POSITION_ALPHA = 0.35;

    /** Hard cap on how many fuels we track at once (sanity limit). */
    public static final int MAX_TRACKED = 20;

    // ==================== State ====================

    private final List<TrackedFuel> trackedFuels = new ArrayList<>();
    private int nextId = 0;

    // ==================== Public API ====================

    /**
     * Feed this cycle's detections into the map.  Call once per periodic().
     *
     * @param fieldPositions Array of field-space positions for every ball
     *                       detected <b>this frame</b>.
     */
    public void update(Translation2d[] fieldPositions) {
        double now = Timer.getFPGATimestamp();

        // Snapshot the count of existing fuels BEFORE adding any new ones.
        // Only these entries participate in matching & decay.
        int existingCount = trackedFuels.size();
        boolean[] matched = new boolean[existingCount];

        for (Translation2d newPos : fieldPositions) {
            // Reject positions that are clearly off the field
            if (newPos.getX() < -0.5 || newPos.getX() > 17.0
                    || newPos.getY() < -0.5 || newPos.getY() > 9.0) {
                continue;
            }

            // Only search among pre-existing fuels (not ones added this frame)
            int bestIdx = -1;
            double bestDist = MERGE_RADIUS_METERS;

            for (int i = 0; i < existingCount; i++) {
                double dist = trackedFuels.get(i).getFieldPosition().getDistance(newPos);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestIdx = i;
                }
            }

            if (bestIdx >= 0) {
                // Merge into existing
                trackedFuels.get(bestIdx).update(newPos, now, POSITION_ALPHA);
                matched[bestIdx] = true;
            } else if (trackedFuels.size() < MAX_TRACKED) {
                // New fuel — appended AFTER the existing entries
                trackedFuels.add(new TrackedFuel(nextId++, newPos, now));
            }
        }

        // Decay unmatched existing fuels & remove expired.
        // Iterate backwards so removal doesn't shift indices we haven't visited.
        for (int i = existingCount - 1; i >= 0; i--) {
            if (!matched[i]) {
                trackedFuels.get(i).decay();
                if (trackedFuels.get(i).isExpired()) {
                    trackedFuels.remove(i);
                }
            }
        }
    }

    /** Returns an unmodifiable snapshot of all currently tracked fuels. */
    public List<TrackedFuel> getTrackedFuels() {
        return List.copyOf(trackedFuels);
    }

    /** Number of fuels currently being tracked. */
    public int size() {
        return trackedFuels.size();
    }

    /** Removes all tracked fuels (e.g. on mode change). */
    public void clear() {
        trackedFuels.clear();
    }
}
