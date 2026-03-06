package frc.robot.subsystems.fueldetection;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;

/**
 * Writes a human-readable CSV debug log to a file on disk every robot cycle.
 * After the sim run, grab the file and paste it into chat for diagnosis.
 *
 * <h2>Output location</h2>
 * {@code fuelhunt_debug.csv} in the project root (working directory).
 *
 * <h2>Columns</h2>
 * cycle, timestamp, robotX, robotY, headingDeg, confirmedFuels, totalTracked,
 * clusterCount, bestX, bestY, bestScore, bestN, bestConf, backFirstDeg,
 * distToBest, trench, activeCmd, simBalls, clusters
 */
public class FuelHuntFileLogger {

    private static final String FILE_NAME = "fuelhunt_debug.csv";

    private static PrintWriter writer = null;
    private static boolean failed = false;
    private static int cycleCount = 0;

    // ==================== Lifecycle ====================

    /**
     * Call once at startup (e.g. in FuelDetection constructor).
     * Uses an absolute path based on the working directory so we know
     * exactly where the file lands.
     */
    public static void init() {
        try {
            File outFile = new File(FILE_NAME).getAbsoluteFile();
            writer = new PrintWriter(new FileWriter(outFile, false)); // overwrite
            writer.println(
                "cycle,timestamp,robotX,robotY,headingDeg,"
                + "confirmed,tracked,clusters,"
                + "bestX,bestY,bestScore,bestN,bestConf,"
                + "bfHeadDeg,distBest,trench,"
                + "cmd,simBalls,details");
            writer.flush();
            System.out.println("[FileLogger] ✓ Opened: " + outFile.getAbsolutePath());
        } catch (IOException e) {
            System.err.println("[FileLogger] ✗ FAILED: " + e.getMessage());
            failed = true;
        }
    }

    // ==================== Per-cycle logging ====================

    /**
     * Call every cycle from {@code FuelDetection.periodic()}.
     * Writes one CSV row with the full snapshot of the fuel-hunt state.
     * Logs every 5th cycle (~10 Hz) to keep file size reasonable.
     */
    public static void logCycle() {
        if (failed || writer == null) return;

        cycleCount++;
        if (cycleCount % 5 != 0) return;

        StringBuilder sb = new StringBuilder();
        try {
            double ts = Timer.getFPGATimestamp();

            // ---- Robot pose (safe even if swerve is null) ----
            double rx = 0, ry = 0, rh = 0;
            try {
                if (RobotContainer.m_swerveInstance != null) {
                    Pose2d p = RobotContainer.m_swerveInstance.getRobotPose();
                    if (p != null) {
                        rx = p.getX();
                        ry = p.getY();
                        rh = p.getRotation().getDegrees();
                    }
                }
            } catch (Exception e) {
                // swerve not ready yet — use zeros
            }

            sb.append(cycleCount).append(',');
            sb.append(f(ts)).append(',');
            sb.append(f(rx)).append(',');
            sb.append(f(ry)).append(',');
            sb.append(String.format("%.1f", rh)).append(',');

            // ---- Fuel map ----
            FuelDetection fuelDet = RobotContainer.m_fuelDetectionInstance;
            if (fuelDet == null || fuelDet.getFuelMap() == null) {
                sb.append("0,0,0,,,,,,,,,none,,");
                writeLine(sb);
                return;
            }

            Translation2d robotPos = new Translation2d(rx, ry);
            FuelMap map = fuelDet.getFuelMap();
            List<TrackedFuel> confirmed = map.getConfirmedFuels();
            List<TrackedFuel> tracked   = map.getTrackedFuels();

            sb.append(confirmed.size()).append(',');
            sb.append(tracked.size()).append(',');

            // ---- Clusters ----
            List<FuelScorer.ScoredCluster> ranked =
                FuelScorer.rankClusters(confirmed, robotPos);
            sb.append(ranked.size()).append(',');

            if (!ranked.isEmpty()) {
                FuelScorer.ScoredCluster best = ranked.get(0);
                FuelCluster bc = best.getCluster();
                Translation2d c = bc.getCentroid();

                sb.append(f(c.getX())).append(',');
                sb.append(f(c.getY())).append(',');
                sb.append(f(best.getScore())).append(',');
                sb.append(bc.getCount()).append(',');
                sb.append(String.format("%.1f", bc.getAverageConfidence())).append(',');

                double ang = Math.atan2(c.getY() - ry, c.getX() - rx);
                sb.append(String.format("%.1f", Math.toDegrees(ang + Math.PI))).append(',');
                sb.append(f(robotPos.getDistance(c))).append(',');
            } else {
                sb.append(",,,,,,,");
            }

            // ---- Trench ----
            double dL = Math.abs(ry - FuelScorer.LEFT_TRENCH_Y);
            double dR = Math.abs(ry - FuelScorer.RIGHT_TRENCH_Y);
            sb.append(dL <= dR ? "L" : "R").append(',');

            // ---- Active command on swerve ----
            try {
                if (RobotContainer.m_swerveInstance != null) {
                    Command cmd = CommandScheduler.getInstance()
                        .requiring(RobotContainer.m_swerveInstance);
                    sb.append(cmd != null ? cmd.getName() : "none");
                } else {
                    sb.append("noSwerve");
                }
            } catch (Exception e) {
                sb.append("err");
            }
            sb.append(',');

            // ---- Sim balls remaining ----
            try {
                if (fuelDet.getSimFuelField() != null) {
                    sb.append(fuelDet.getSimFuelField().getRemainingCount());
                } else {
                    sb.append("-");
                }
            } catch (Exception e) {
                sb.append("err");
            }
            sb.append(',');

            // ---- Top 5 cluster details ----
            sb.append('"');
            int limit = Math.min(ranked.size(), 5);
            for (int i = 0; i < limit; i++) {
                FuelScorer.ScoredCluster sc = ranked.get(i);
                FuelCluster fc = sc.getCluster();
                if (i > 0) sb.append(" | ");
                sb.append(String.format("#%d(%.1f,%.1f)s=%.2f n=%d",
                    i,
                    fc.getCentroid().getX(), fc.getCentroid().getY(),
                    sc.getScore(), fc.getCount()));
            }
            sb.append('"');

            writeLine(sb);

        } catch (Exception e) {
            // Last-resort safety net — log the error once, then keep going
            System.err.println("[FileLogger] logCycle error: " + e.getMessage());
        }
    }

    // ==================== Shutdown ====================

    /** Call on robot disable to flush and close the file cleanly. */
    public static void close() {
        if (writer != null) {
            writer.flush();
            writer.close();
            writer = null;
            System.out.println("[FileLogger] ✓ Closed: " + new File(FILE_NAME).getAbsolutePath()
                + "  (" + cycleCount + " cycles)");
        }
    }

    // ==================== Internals ====================

    private static void writeLine(StringBuilder sb) {
        writer.println(sb.toString());
        writer.flush();
    }

    private static String f(double v) {
        return String.format("%.3f", v);
    }
}
