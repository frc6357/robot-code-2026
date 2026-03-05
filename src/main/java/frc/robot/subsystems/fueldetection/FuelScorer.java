package frc.robot.subsystems.fueldetection;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Clusters confirmed fuels and scores each cluster to determine the best
 * target for the robot to drive toward.
 *
 * <h2>Scoring factors</h2>
 * <ol>
 *   <li><b>Group density</b> — more balls in one area = higher score</li>
 *   <li><b>Proximity</b> — closer clusters are cheaper to reach</li>
 *   <li><b>Path cost</b> — clusters near a trench line are cheaper to
 *       return through, so the robot doesn't waste time crossing the field</li>
 * </ol>
 *
 * All weights are public constants so they can be tuned on the dashboard.
 */
public class FuelScorer {

    // ==================== Clustering constants ====================

    /**
     * Maximum distance (meters) between two fuels for them to be in the same
     * cluster.  This should be roughly the intake sweep width of a single
     * drive pass — balls within this radius can be collected together.
     */
    public static final double CLUSTER_RADIUS_M = 1.0;

    // ==================== Scoring weights ====================

    /** Weight for how many fuels are in the cluster. */
    public static final double W_DENSITY   = 3.0;

    /** Weight for proximity (inverse distance to robot). */
    public static final double W_PROXIMITY = 2.0;

    /** Weight for path-cost efficiency (closeness to a trench Y-line). */
    public static final double W_PATH_COST = 1.5;

    /** Weight for average confidence of the cluster. */
    public static final double W_CONFIDENCE = 0.5;

    // ==================== Field geometry ====================

    /** Y coordinate of the left trench center line (meters). */
    public static final double LEFT_TRENCH_Y  = 0.615;

    /** Y coordinate of the right trench center line (meters). */
    public static final double RIGHT_TRENCH_Y = 7.428;

    /** Half-width of a trench — fuels within this band are "on" the trench. */
    public static final double TRENCH_HALF_WIDTH = 1.0;

    // ==================== Public API ====================

    /**
     * Builds clusters from the confirmed fuel list, scores them relative to
     * the robot's current position, and returns them sorted best-first.
     *
     * @param confirmedFuels Only fuels that have passed the confidence
     *                       threshold (use {@link FuelMap#getConfirmedFuels()}).
     * @param robotPosition  Current robot field-space position.
     * @return Scored clusters, sorted descending by score (index 0 = best).
     *         Empty list if no confirmed fuels exist.
     */
    public static List<ScoredCluster> rankClusters(
            List<TrackedFuel> confirmedFuels,
            Translation2d robotPosition) {

        if (confirmedFuels.isEmpty()) {
            return List.of();
        }

        // 1. Cluster
        List<FuelCluster> clusters = buildClusters(confirmedFuels);

        // 2. Score each cluster
        List<ScoredCluster> scored = new ArrayList<>();
        for (FuelCluster c : clusters) {
            double score = scoreCluster(c, robotPosition);
            scored.add(new ScoredCluster(c, score));
        }

        // 3. Sort descending by score
        scored.sort(Comparator.comparingDouble(ScoredCluster::getScore).reversed());
        return scored;
    }

    /**
     * Convenience — returns the best cluster target position, or empty if no
     * confirmed fuels exist.
     */
    public static java.util.Optional<FuelCluster> bestCluster(
            List<TrackedFuel> confirmedFuels,
            Translation2d robotPosition) {

        List<ScoredCluster> ranked = rankClusters(confirmedFuels, robotPosition);
        if (ranked.isEmpty()) return java.util.Optional.empty();
        return java.util.Optional.of(ranked.get(0).getCluster());
    }

    // ==================== Clustering ====================

    /**
     * Simple single-linkage clustering: each fuel is assigned to the first
     * cluster whose centroid is within {@link #CLUSTER_RADIUS_M}.  If none
     * match, a new cluster is started.
     *
     * <p>This is intentionally simple and O(n·k) — with ≤50 confirmed fuels
     * the cost is negligible.
     */
    static List<FuelCluster> buildClusters(List<TrackedFuel> fuels) {
        // Temp mutable lists — one per cluster-in-progress
        List<List<TrackedFuel>> clusterMembers = new ArrayList<>();
        List<Translation2d> clusterCentroids = new ArrayList<>();

        for (TrackedFuel fuel : fuels) {
            Translation2d pos = fuel.getFieldPosition();
            int bestCluster = -1;
            double bestDist = CLUSTER_RADIUS_M;

            for (int i = 0; i < clusterCentroids.size(); i++) {
                double dist = clusterCentroids.get(i).getDistance(pos);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestCluster = i;
                }
            }

            if (bestCluster >= 0) {
                // Add to existing cluster and recompute centroid
                List<TrackedFuel> members = clusterMembers.get(bestCluster);
                members.add(fuel);
                clusterCentroids.set(bestCluster, computeCentroid(members));
            } else {
                // Start a new cluster
                List<TrackedFuel> newMembers = new ArrayList<>();
                newMembers.add(fuel);
                clusterMembers.add(newMembers);
                clusterCentroids.add(pos);
            }
        }

        // Wrap into FuelCluster objects
        List<FuelCluster> result = new ArrayList<>();
        for (List<TrackedFuel> members : clusterMembers) {
            result.add(new FuelCluster(members));
        }
        return result;
    }

    private static Translation2d computeCentroid(List<TrackedFuel> members) {
        double sx = 0, sy = 0;
        for (TrackedFuel f : members) {
            sx += f.getFieldPosition().getX();
            sy += f.getFieldPosition().getY();
        }
        return new Translation2d(sx / members.size(), sy / members.size());
    }

    // ==================== Scoring ====================

    /**
     * Computes a composite score for a single cluster.
     *
     * <pre>
     *   score = W_DENSITY   × densityScore
     *         + W_PROXIMITY × proximityScore
     *         + W_PATH_COST × pathCostScore
     *         + W_CONFIDENCE × confidenceScore
     * </pre>
     *
     * Each sub-score is normalized to roughly [0, 1].
     */
    static double scoreCluster(FuelCluster cluster, Translation2d robotPos) {
        double density    = densityScore(cluster);
        double proximity  = proximityScore(cluster, robotPos);
        double pathCost   = pathCostScore(cluster);
        double confidence = confidenceScore(cluster);

        return W_DENSITY   * density
             + W_PROXIMITY * proximity
             + W_PATH_COST * pathCost
             + W_CONFIDENCE * confidence;
    }

    /**
     * More fuels = higher score.  Saturates at 5 balls
     * (diminishing returns beyond that).
     */
    private static double densityScore(FuelCluster c) {
        return Math.min(c.getCount() / 5.0, 1.0);
    }

    /**
     * Closer = higher score.  Uses an inverse-distance curve
     * that gives 1.0 at 0 m and ≈0.25 at 6 m.
     */
    private static double proximityScore(FuelCluster c, Translation2d robotPos) {
        double dist = robotPos.getDistance(c.getCentroid());
        // Logistic-style falloff: 1 / (1 + dist/2)
        return 1.0 / (1.0 + dist / 2.0);
    }

    /**
     * Clusters near a trench center line are cheaper to return through.
     * Score is 1.0 if centroid is on the trench line, falling off to ~0.3
     * at the midfield center (Y ≈ 4.0).
     */
    private static double pathCostScore(FuelCluster c) {
        double y = c.getCentroid().getY();
        double distToLeft  = Math.abs(y - LEFT_TRENCH_Y);
        double distToRight = Math.abs(y - RIGHT_TRENCH_Y);
        double distToNearest = Math.min(distToLeft, distToRight);

        // Normalize: 0 distance → 1.0, TRENCH_HALF_WIDTH → 0.5, far → approaches 0
        return 1.0 / (1.0 + distToNearest / TRENCH_HALF_WIDTH);
    }

    /**
     * Higher average confidence = more likely the balls actually exist.
     * Normalized against MAX_CONFIDENCE.
     */
    private static double confidenceScore(FuelCluster c) {
        return c.getAverageConfidence() / TrackedFuel.MAX_CONFIDENCE;
    }

    // ==================== Result wrapper ====================

    /** A cluster paired with its computed score. */
    public static class ScoredCluster {
        private final FuelCluster cluster;
        private final double score;

        public ScoredCluster(FuelCluster cluster, double score) {
            this.cluster = cluster;
            this.score = score;
        }

        public FuelCluster getCluster() { return cluster; }
        public double getScore()        { return score; }

        @Override
        public String toString() {
            return String.format("ScoredCluster[score=%.2f, %s]", score, cluster);
        }
    }
}
