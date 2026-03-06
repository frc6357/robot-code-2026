package frc.robot.subsystems.fueldetection;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Simulated fuel field that places 200 balls randomly scattered across the
 * neutral zone and feeds them into the {@link FuelMap} each cycle based on
 * which balls are "visible" from the robot's camera.
 *
 * <h2>Usage</h2>
 * In simulation mode, call {@link #getVisibleFuelPositions(Pose2d)} every
 * cycle instead of reading the real Limelight.  The returned
 * {@code Translation2d[]} can be fed directly into {@link FuelMap#update}.
 *
 * <h2>Ball layout (200 total)</h2>
 * All 200 balls are uniformly randomly distributed across the neutral zone
 * (X: 5.0–12.0, Y: 0.5–7.5).
 *
 * <h2>Camera model</h2>
 * A simple cone-shaped FOV is used: balls within {@link #MAX_RANGE_M} and
 * within ±{@link #HALF_FOV_DEG} of the camera's forward direction are
 * considered visible.  No occlusion is simulated.
 */
public class SimFuelField {

    // ==================== Field geometry ====================

    /** Neutral zone X min (meters). */
    private static final double NZ_MIN_X = 5.0;
    /** Neutral zone X max (meters). */
    private static final double NZ_MAX_X = 12.0;
    /** Neutral zone Y min (meters). */
    private static final double NZ_MIN_Y = 0.5;
    /** Neutral zone Y max (meters). */
    private static final double NZ_MAX_Y = 7.5;

    /** Number of randomly placed balls. */
    private static final int BALL_COUNT = 200;

    // ==================== Camera FOV model ====================

    /** Maximum range a ball can be "seen" from the camera, in meters. */
    private static final double MAX_RANGE_M = 5.0;

    /** Half of the horizontal field-of-view of the simulated camera (degrees). */
    private static final double HALF_FOV_DEG = 30.0;

    // ==================== Ball collection ====================

    /**
     * When the robot gets closer than this to a ball, the ball is "collected"
     * (removed from the field).  Set to approximate intake radius.
     */
    private static final double COLLECT_RADIUS_M = 0.50;

    // ==================== State ====================

    private final List<Translation2d> balls = new ArrayList<>();

    // ==================== Constructor ====================

    /**
     * Creates the simulated field with 200 balls randomly scattered
     * across the neutral zone.
     */
    public SimFuelField() {
        Random rng = new Random(2026); // fixed seed for reproducibility

        for (int i = 0; i < BALL_COUNT; i++) {
            double x = NZ_MIN_X + rng.nextDouble() * (NZ_MAX_X - NZ_MIN_X);
            double y = NZ_MIN_Y + rng.nextDouble() * (NZ_MAX_Y - NZ_MIN_Y);
            balls.add(new Translation2d(x, y));
        }
    }

    // ==================== Public API ====================

    /**
     * Returns the field positions of all balls that are currently "visible"
     * from the robot's camera.  Also removes any ball the robot has driven
     * over (collected).
     *
     * @param robotPose Current simulated robot pose
     * @return Array of field-space Translation2d for each visible ball
     */
    public Translation2d[] getVisibleFuelPositions(Pose2d robotPose) {
        Translation2d robotPos = robotPose.getTranslation();
        double robotHeadingRad = robotPose.getRotation().getRadians();
        double halfFovRad = Math.toRadians(HALF_FOV_DEG);

        // ---- Remove collected balls ----
        balls.removeIf(ball -> ball.getDistance(robotPos) < COLLECT_RADIUS_M);

        // ---- Determine visible balls ----
        List<Translation2d> visible = new ArrayList<>();

        for (Translation2d ball : balls) {
            double dx = ball.getX() - robotPos.getX();
            double dy = ball.getY() - robotPos.getY();
            double distance = Math.hypot(dx, dy);

            if (distance > MAX_RANGE_M || distance < 0.15) {
                continue; // too far or too close
            }

            // Angle from robot heading to the ball
            double angleToBall = Math.atan2(dy, dx);
            double relativeAngle = angleToBall - robotHeadingRad;

            // Normalize to [-π, π]
            relativeAngle = Math.atan2(Math.sin(relativeAngle), Math.cos(relativeAngle));

            if (Math.abs(relativeAngle) <= halfFovRad) {
                // Add a tiny bit of noise to simulate real-world projection jitter
                double noiseX = (Math.random() - 0.5) * 0.03;
                double noiseY = (Math.random() - 0.5) * 0.03;
                visible.add(new Translation2d(ball.getX() + noiseX, ball.getY() + noiseY));
            }
        }

        return visible.toArray(new Translation2d[0]);
    }

    /**
     * Returns the total number of balls remaining on the field.
     */
    public int getRemainingCount() {
        return balls.size();
    }

    /**
     * Returns a copy of all ball positions (for logging / display).
     */
    public List<Translation2d> getAllBallPositions() {
        return List.copyOf(balls);
    }

    /**
     * Resets the field to its original layout (e.g. on autonomous init).
     */
    public void reset() {
        balls.clear();
        // Re-run the same placement logic via a new instance trick
        SimFuelField fresh = new SimFuelField();
        balls.addAll(fresh.balls);
    }
}
