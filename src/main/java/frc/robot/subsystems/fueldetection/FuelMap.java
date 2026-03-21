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
     * <p>This overload has <b>no camera frustum</b> — all unmatched fuels
     * decay every frame.  Prefer the frustum-aware overload
     * {@link #update(Translation2d[], Translation2d, double, double, double)}
     * so fuels outside the camera's view persist correctly.
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
        update(fieldPositions, null, 0, 0, 0);
    }

    /**
     * Feed this cycle's detections into the map with camera frustum
     * information for visibility-aware decay.
     *
     * <p>Only fuels that fall <b>inside</b> the camera's field of view
     * (defined by {@code cameraOrigin}, {@code cameraHeadingRad}, and
     * {@code halfFovRad}) are decayed when not matched.  Fuels outside the
     * FOV are left at their current confidence — they persist until the
     * camera looks at them again.
     *
     * @param fieldPositions    Array of field-space positions for every ball
     *                          detected this frame.
     * @param cameraOrigin      Field-space position of the camera (robot pose
     *                          + camera offset).  {@code null} to fall back to
     *                          decay-all behaviour.
     * @param cameraHeadingRad  Camera heading in field space (radians).
     * @param halfFovRad        Half the horizontal FOV of the camera (radians).
     * @param maxRange          Maximum detection range of the camera (meters).
     */
    public void update(Translation2d[] fieldPositions,
                       Translation2d cameraOrigin,
                       double cameraHeadingRad,
                       double halfFovRad,
                       double maxRange) {
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
                // Only decay fuels that are inside the camera's FOV.
                // Fuels outside the FOV are simply not visible — we have
                // no new evidence about them, so leave their confidence
                // unchanged until the camera looks at them again.
                if (isInsideFov(trackedFuels.get(i).getFieldPosition(),
                                cameraOrigin, cameraHeadingRad,
                                halfFovRad, maxRange)) {
                    trackedFuels.get(i).decay();
                    if (trackedFuels.get(i).isExpired()) {
                        trackedFuels.remove(i);
                    }
                }
            }
        }
    }

    // ==================== Internals ====================

    /**
     * Determines whether a field position falls inside the camera's
     * cone-shaped field of view.
     *
     * <p>If {@code cameraOrigin} is {@code null}, this returns {@code true}
     * so that all fuels decay (legacy behaviour / fallback).
     *
     * @param fuelPos           Field-space position to test
     * @param cameraOrigin      Field-space origin of the camera
     * @param cameraHeadingRad  Camera heading in field space (radians)
     * @param halfFovRad        Half horizontal FOV (radians)
     * @param maxRange          Maximum detection range (meters)
     * @return true if the fuel is inside the view cone (or no frustum info)
     */
    private boolean isInsideFov(Translation2d fuelPos,
                                Translation2d cameraOrigin,
                                double cameraHeadingRad,
                                double halfFovRad,
                                double maxRange) {
        if (cameraOrigin == null) {
            return true; // no frustum data — fall back to decay-all
        }

        double dx = fuelPos.getX() - cameraOrigin.getX();
        double dy = fuelPos.getY() - cameraOrigin.getY();
        double dist = Math.hypot(dx, dy);

        // Beyond camera range — not visible
        if (dist > maxRange || dist < 0.15) {
            return false;
        }

        // Angle from camera heading to the fuel
        double angleToBall = Math.atan2(dy, dx);
        double relativeAngle = angleToBall - cameraHeadingRad;
        // Normalize to [-π, π]
        relativeAngle = Math.atan2(Math.sin(relativeAngle), Math.cos(relativeAngle));

        return Math.abs(relativeAngle) <= halfFovRad;
    }

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
}
