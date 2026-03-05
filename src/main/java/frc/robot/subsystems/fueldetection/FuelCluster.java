package frc.robot.subsystems.fueldetection;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A group of nearby {@link TrackedFuel} entries that are spatially close enough
 * to be collected together in a single drive pass.
 */
public class FuelCluster {

    /** The centroid (average position) of all fuels in this cluster. */
    private final Translation2d centroid;

    /** Number of fuels in this cluster. */
    private final int count;

    /** Sum of confidence values across all fuels in the cluster. */
    private final int totalConfidence;

    /** The individual fuels that make up this cluster. */
    private final List<TrackedFuel> members;

    public FuelCluster(List<TrackedFuel> members) {
        this.members = List.copyOf(members);
        this.count = members.size();

        double sumX = 0, sumY = 0;
        int sumConf = 0;
        for (TrackedFuel f : members) {
            sumX += f.getFieldPosition().getX();
            sumY += f.getFieldPosition().getY();
            sumConf += f.getConfidence();
        }

        this.centroid = new Translation2d(sumX / count, sumY / count);
        this.totalConfidence = sumConf;
    }

    public Translation2d getCentroid()      { return centroid; }
    public int getCount()                   { return count; }
    public int getTotalConfidence()         { return totalConfidence; }
    public List<TrackedFuel> getMembers()   { return members; }

    /**
     * Average confidence per fuel — a proxy for how reliably these balls
     * actually exist (vs. ghost detections).
     */
    public double getAverageConfidence() {
        return (double) totalConfidence / count;
    }

    @Override
    public String toString() {
        return String.format("FuelCluster[n=%d, centroid=(%.2f,%.2f), avgConf=%.1f]",
                count, centroid.getX(), centroid.getY(), getAverageConfidence());
    }
}
