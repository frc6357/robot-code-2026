package frc.robot.subsystems.launcher.moveandshoot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import frc.lib.utils.FieldConstants;

/**
 * Calculates the optimal aim point along the hexagonal rim of the HUB scoring target
 * based on the robot's current field position.
 *
 * <p>The HUB has a hexagonal opening. Rather than discretely picking vertices (which causes
 * discontinuous jumps), this class computes a <b>continuous</b> aim point on the hex perimeter
 * that moves smoothly as the robot moves around the field.
 *
 * <p>Algorithm:
 * <ol>
 *   <li>Compute the bearing from the hub center to the robot — this angle is continuous</li>
 *   <li>The "far edge" of the hex (opposite the robot) is found by intersecting a ray in the
 *       opposite direction with the hex boundary. This intersection slides smoothly along
 *       hex edges and through vertices as the robot orbits the hub.</li>
 *   <li>The aim point is interpolated between the hub center and the far-edge point using
 *       {@link #AIM_WEIGHT}, biasing toward the far side for more scoring margin.</li>
 * </ol>
 *
 * <p>All methods are static — no instantiation required.
 *
 * <p>Usage:
 * <pre>{@code
 * Translation3d aimPoint = HubTargetingSystem.getAimPoint(robotPose, false);
 * double shooterYaw = HubTargetingSystem.getAimYaw(robotPose, false).getRadians();
 * double distance = HubTargetingSystem.getAimDistance(robotPose, false);
 * }</pre>
 */
public class HubTargetingSystem {

    /** Private constructor — utility class, do not instantiate. */
    private HubTargetingSystem() {}

    // ===== Tunable Constants =====

    /** Center of the blue-side hub in field coordinates. */
    private static final Translation2d BLUE_HUB_CENTER = FieldConstants.Hub.topCenterPoint.toTranslation2d();

    /** Center of the red-side hub in field coordinates. */
    private static final Translation2d RED_HUB_CENTER = FieldConstants.Hub.redTopCenterPoint.toTranslation2d();

    /**
     * Circumradius of the hexagonal opening (center to vertex).
     * Derived from the flat-to-flat inner width: r = (innerWidth / 2) / cos(π/6).
     * FieldConstants.Hub.innerWidth is already in meters.
     */
    private static final double HEX_CIRCUMRADIUS =
        (FieldConstants.Hub.innerWidth / 2.0) / Math.cos(Math.PI / 6.0);

    /**
     * Rotation offset of the hexagon relative to the field X-axis, in radians.
     * 0 means one flat side faces the positive X direction (i.e. a vertex points at +X).
     * Adjust if the physical hex is rotated.
     */
    private static final double HEX_ROTATION_OFFSET_RAD = 0.0;

    /** Number of vertices on the hexagon. */
    private static final int VERTEX_COUNT = 6;

    /**
     * Weight for biasing the aim point away from the hub center toward the far edge.
     * 0.0 = aim at hub center, 1.0 = aim at the far edge of the hex perimeter.
     * A value >0.5 biases toward the far side for more scoring margin.
     */
    private static final double AIM_WEIGHT = 0.65;

    /** Height of the hub opening rim, used as the Z component of the 3D aim point. */
    private static final double HUB_HEIGHT = FieldConstants.Hub.height;

    /**
     * Extra inward offset toward the hub center, in meters.
     * Shifts the aim point ~4 inches closer to the center of the hub for better scoring margin.
     */
    private static final double INWARD_OFFSET_METERS = 0.14; // 4 inches

    // ===== Precomputed Vertices =====

    private static final List<Translation2d> blueVertices =
        Collections.unmodifiableList(computeVertices(BLUE_HUB_CENTER));
    private static final List<Translation2d> redVertices =
        Collections.unmodifiableList(computeVertices(RED_HUB_CENTER));

    /**
     * Computes the 6 hex vertex positions in field space for a given hub center.
     *
     * @param center The center of the hub
     * @return List of 6 vertex positions
     */
    private static List<Translation2d> computeVertices(Translation2d center) {
        List<Translation2d> vertices = new ArrayList<>(VERTEX_COUNT);
        for (int i = 0; i < VERTEX_COUNT; i++) {
            double angle = HEX_ROTATION_OFFSET_RAD + (2.0 * Math.PI * i / VERTEX_COUNT);
            vertices.add(new Translation2d(
                center.getX() + HEX_CIRCUMRADIUS * Math.cos(angle),
                center.getY() + HEX_CIRCUMRADIUS * Math.sin(angle)
            ));
        }
        return vertices;
    }

    /**
     * Returns the 6 hex vertices for the specified alliance hub, for field visualization.
     *
     * @param useRedHub true to get the red hub vertices, false for blue
     * @return Unmodifiable list of 6 vertex Translation2d positions in field space
     */
    public static List<Translation2d> getHexVertices(boolean useRedHub) {
        return useRedHub ? redVertices : blueVertices;
    }

    /**
     * Computes the optimal aim point on the hex rim for the given robot pose.
     *
     * <p>The aim point moves continuously along the hex perimeter as the robot
     * moves, avoiding discrete jumps that would cause oscillations in PID/BangBang loops.
     *
     * @param robotPose The robot's current field pose
     * @param useRedHub true to aim at the red hub, false for blue
     * @return The optimal aim point as a Translation3d in field coordinates (Z = hub height)
     */
    public static Translation3d getAimPoint(Pose2d robotPose, boolean useRedHub) {
        Translation2d aimPoint2d = getAimPoint2d(robotPose, useRedHub);
        logDebugInfo(robotPose, useRedHub);
        return new Translation3d(aimPoint2d.getX(), aimPoint2d.getY(), HUB_HEIGHT);
    }

    /**
     * Computes the optimal aim point on the blue hub for the given robot pose.
     *
     * @param robotPose The robot's current field pose
     * @return The optimal aim point as a Translation3d targeting the blue hub
     */
    public static Translation3d getAimPointBlue(Pose2d robotPose) {
        return getAimPoint(robotPose, false);
    }

    /**
     * Computes the optimal aim point on the red hub for the given robot pose.
     *
     * @param robotPose The robot's current field pose
     * @return The optimal aim point as a Translation3d targeting the red hub
     */
    public static Translation3d getAimPointRed(Pose2d robotPose) {
        return getAimPoint(robotPose, true);
    }

    /**
     * Internal 2D aim point calculation using continuous hex perimeter sampling.
     *
     * <p>Computes the bearing from the hub center to the robot, then finds the point on the
     * hex perimeter in the <b>opposite</b> direction (the "far edge"). The aim point is
     * interpolated between the hub center and this far-edge point. Because the hex perimeter
     * is a continuous function of angle, the resulting aim point moves smoothly as the robot
     * moves — no discrete jumps at vertex boundaries.
     *
     * @param robotPose The robot's current field pose
     * @param useRedHub true to aim at the red hub, false for blue
     * @return The optimal aim point as a Translation2d in field coordinates
     */
    private static Translation2d getAimPoint2d(Pose2d robotPose, boolean useRedHub) {
        Translation2d robotPos = robotPose.getTranslation();
        Translation2d hubCenter = useRedHub ? RED_HUB_CENTER : BLUE_HUB_CENTER;

        // Bearing from hub center to robot (continuous as the robot moves)
        double bearingToRobot = Math.atan2(
            robotPos.getY() - hubCenter.getY(),
            robotPos.getX() - hubCenter.getX()
        );

        // The far edge is the opposite direction from the robot
        double farEdgeAngle = bearingToRobot + Math.PI;

        // Find the point on the hex perimeter at this angle
        Translation2d farEdgePoint = hexPerimeterPoint(hubCenter, farEdgeAngle);

        // Interpolate between hub center (0.0) and far edge point (1.0)
        Translation2d interpolated = hubCenter.interpolate(farEdgePoint, AIM_WEIGHT);

        // Shift the aim point inward toward the hub center by INWARD_OFFSET_METERS
        double dx = hubCenter.getX() - interpolated.getX();
        double dy = hubCenter.getY() - interpolated.getY();
        double dist = Math.hypot(dx, dy);
        if (dist > 1e-6) {
            interpolated = new Translation2d(
                interpolated.getX() + (dx / dist) * INWARD_OFFSET_METERS,
                interpolated.getY() + (dy / dist) * INWARD_OFFSET_METERS
            );
        }

        return interpolated;
    }

    /**
     * Computes the point on the hex perimeter at a given angle from center.
     *
     * <p>This is the key function that ensures continuity. For any angle θ, we determine
     * which edge of the hexagon a ray from center at angle θ intersects, then compute
     * the exact intersection point. As θ changes continuously, the intersection point
     * slides smoothly along edges and through vertices.
     *
     * <p>The hex is defined by vertices at angles {@code HEX_ROTATION_OFFSET_RAD + k·π/3}
     * for k = 0..5, each at distance {@link #HEX_CIRCUMRADIUS} from center.
     *
     * @param center The hub center position
     * @param angle  The angle (radians, field-frame) to sample
     * @return The point on the hex perimeter in field coordinates
     */
    private static Translation2d hexPerimeterPoint(Translation2d center, double angle) {
        double sectorWidth = Math.PI / 3.0; // 60° per sector

        // Normalize the angle relative to the hex rotation into [0, 2π)
        double localAngle = angle - HEX_ROTATION_OFFSET_RAD;
        localAngle = localAngle % (2.0 * Math.PI);
        if (localAngle < 0) localAngle += 2.0 * Math.PI;

        // Each sector spans 60°. Vertex i is at angle i*60°.
        // The edge between vertex i and vertex (i+1) spans [i*60°, (i+1)*60°].
        int sectorIndex = (int) (localAngle / sectorWidth);
        if (sectorIndex >= VERTEX_COUNT) sectorIndex = VERTEX_COUNT - 1;

        // The two vertex angles bounding this sector
        double angle1 = HEX_ROTATION_OFFSET_RAD + sectorIndex * sectorWidth;
        double angle2 = HEX_ROTATION_OFFSET_RAD + (sectorIndex + 1) * sectorWidth;

        // The midpoint of the edge is at the average of the two vertex angles.
        // The perpendicular distance from center to this edge is the apothem.
        // The distance from center to the perimeter along any ray within this sector is:
        //   r(θ) = apothem / cos(θ - sectorMidAngle)
        double sectorMidAngle = (angle1 + angle2) / 2.0;
        double angleFromMid = MathUtil.angleModulus(angle - sectorMidAngle);

        // Clamp to avoid division issues at extreme angles (should not happen in practice)
        double cosAngle = Math.cos(angleFromMid);
        if (Math.abs(cosAngle) < 1e-9) cosAngle = Math.copySign(1e-9, cosAngle);

        // The apothem (center to edge midpoint) = circumradius * cos(π/6)
        double apothem = HEX_CIRCUMRADIUS * Math.cos(Math.PI / 6.0);
        double perimeterRadius = apothem / cosAngle;

        return new Translation2d(
            center.getX() + perimeterRadius * Math.cos(angle),
            center.getY() + perimeterRadius * Math.sin(angle)
        );
    }

    /**
     * Computes the field-relative yaw angle the robot (or turret) must aim at to hit
     * the optimal aim point.
     *
     * @param robotPose The robot's current field pose
     * @param useRedHub true to aim at the red hub, false for blue
     * @return The bearing to the aim point as a Rotation2d
     */
    public static Rotation2d getAimYaw(Pose2d robotPose, boolean useRedHub) {
        Translation2d aimPoint = getAimPoint2d(robotPose, useRedHub);
        Translation2d robotPos = robotPose.getTranslation();
        return new Rotation2d(
            aimPoint.getX() - robotPos.getX(),
            aimPoint.getY() - robotPos.getY()
        );
    }

    /**
     * Computes the distance from the robot to the optimal aim point.
     * Useful for flywheel speed and hood angle lookup tables.
     *
     * @param robotPose The robot's current field pose
     * @param useRedHub true to aim at the red hub, false for blue
     * @return Distance in meters
     */
    public static double getAimDistance(Pose2d robotPose, boolean useRedHub) {
        return robotPose.getTranslation().getDistance(getAimPoint2d(robotPose, useRedHub));
    }

    /**
     * Logs all targeting debug info via AdvantageKit Logger.
     * Called automatically by {@link #getAimPoint(Pose2d, boolean)}.
     *
     * @param robotPose The robot's current field pose
     * @param useRedHub true to target the red hub, false for blue
     */
    private static void logDebugInfo(Pose2d robotPose, boolean useRedHub) {
        Translation2d robotPos = robotPose.getTranslation();
        Translation2d hubCenter = useRedHub ? RED_HUB_CENTER : BLUE_HUB_CENTER;
        Translation2d aimPoint = getAimPoint2d(robotPose, useRedHub);

        // Bearing from hub center to robot (the continuous input to the algorithm)
        double bearingToRobot = Math.atan2(
            robotPos.getY() - hubCenter.getY(),
            robotPos.getX() - hubCenter.getX()
        );
        double farEdgeAngle = bearingToRobot + Math.PI;
        Translation2d farEdgePoint = hexPerimeterPoint(hubCenter, farEdgeAngle);

        // Bearing from robot to hub center (for turret reference)
        double bearingToCenter = Math.toDegrees(Math.atan2(
            hubCenter.getY() - robotPos.getY(),
            hubCenter.getX() - robotPos.getX()
        ));

        String prefix = "HubTargeting/";

        Logger.recordOutput(prefix + "Alliance", useRedHub ? "RED" : "BLUE");
        Logger.recordOutput(prefix + "HubBearing(deg)", bearingToCenter);
        Logger.recordOutput(prefix + "FarEdgeAngle(deg)", Math.toDegrees(farEdgeAngle));
        Logger.recordOutput(prefix + "FarEdgePoint", farEdgePoint);
        Logger.recordOutput(prefix + "FarEdgeDist(m)", robotPos.getDistance(farEdgePoint));
        Logger.recordOutput(prefix + "AimPoint", aimPoint);
        Logger.recordOutput(prefix + "AimPoint3d",
            new Translation3d(aimPoint.getX(), aimPoint.getY(), HUB_HEIGHT));
        Logger.recordOutput(prefix + "AimDist(m)", robotPos.getDistance(aimPoint));
        Logger.recordOutput(prefix + "AimYaw(deg)", Math.toDegrees(Math.atan2(
            aimPoint.getY() - robotPos.getY(),
            aimPoint.getX() - robotPos.getX()
        )));
    }
}
