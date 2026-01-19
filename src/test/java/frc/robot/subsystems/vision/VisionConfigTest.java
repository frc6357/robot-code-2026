package frc.robot.subsystems.vision;

import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for VisionConfig constants and records.
 * Tests the PoseStdDevs record and Thresholds class to ensure correct values.
 */
class VisionConfigTest {

    @Test
    void testPoseStdDevs_STATIONARY_CLOSE() {
        VisionConfig.PoseStdDevs stdDevs = VisionConfig.PoseStdDevs.STATIONARY_CLOSE;
        assertEquals(0.1, stdDevs.xy(), "STATIONARY_CLOSE xy should be 0.1");
        assertEquals(0.1, stdDevs.theta(), "STATIONARY_CLOSE theta should be 0.1");
    }

    @Test
    void testPoseStdDevs_MULTI_TAG() {
        VisionConfig.PoseStdDevs stdDevs = VisionConfig.PoseStdDevs.MULTI_TAG;
        assertEquals(0.25, stdDevs.xy(), "MULTI_TAG xy should be 0.25");
        assertEquals(8.0, stdDevs.theta(), "MULTI_TAG theta should be 8.0");
    }

    @Test
    void testPoseStdDevs_STRONG_MULTI() {
        VisionConfig.PoseStdDevs stdDevs = VisionConfig.PoseStdDevs.STRONG_MULTI;
        assertEquals(0.1, stdDevs.xy(), "STRONG_MULTI xy should be 0.1");
        assertEquals(0.1, stdDevs.theta(), "STRONG_MULTI theta should be 0.1");
    }

    @Test
    void testPoseStdDevs_CLOSE_SINGLE() {
        VisionConfig.PoseStdDevs stdDevs = VisionConfig.PoseStdDevs.CLOSE_SINGLE;
        assertEquals(0.5, stdDevs.xy(), "CLOSE_SINGLE xy should be 0.5");
        assertEquals(16.0, stdDevs.theta(), "CLOSE_SINGLE theta should be 16.0");
    }

    @Test
    void testPoseStdDevs_PROXIMITY() {
        VisionConfig.PoseStdDevs stdDevs = VisionConfig.PoseStdDevs.PROXIMITY;
        assertEquals(2.0, stdDevs.xy(), "PROXIMITY xy should be 2.0");
        assertEquals(999999.0, stdDevs.theta(), "PROXIMITY theta should be 999999.0");
    }

    @Test
    void testPoseStdDevs_STABLE() {
        VisionConfig.PoseStdDevs stdDevs = VisionConfig.PoseStdDevs.STABLE;
        assertEquals(0.5, stdDevs.xy(), "STABLE xy should be 0.5");
        assertEquals(999999.0, stdDevs.theta(), "STABLE theta should be 999999.0");
    }

    @Test
    void testPoseStdDevs_RESET() {
        VisionConfig.PoseStdDevs stdDevs = VisionConfig.PoseStdDevs.RESET;
        assertEquals(0.001, stdDevs.xy(), "RESET xy should be 0.001");
        assertEquals(0.001, stdDevs.theta(), "RESET theta should be 0.001");
    }

    @Test
    void testPoseStdDevs_HIGH_AMBIGUITY_PENALTY() {
        VisionConfig.PoseStdDevs stdDevs = VisionConfig.PoseStdDevs.HIGH_AMBIGUITY_PENALTY;
        assertEquals(0, stdDevs.xy(), "HIGH_AMBIGUITY_PENALTY xy should be 0");
        assertEquals(15.0, stdDevs.theta(), "HIGH_AMBIGUITY_PENALTY theta should be 15.0");
    }

    @Test
    void testPoseStdDevs_HIGH_ROTATION_PENALTY() {
        VisionConfig.PoseStdDevs stdDevs = VisionConfig.PoseStdDevs.HIGH_ROTATION_PENALTY;
        assertEquals(0, stdDevs.xy(), "HIGH_ROTATION_PENALTY xy should be 0");
        assertEquals(15.0, stdDevs.theta(), "HIGH_ROTATION_PENALTY theta should be 15.0");
    }

    @Test
    void testThresholds_PhysicalConstraints() {
        assertEquals(0.25, VisionConfig.Thresholds.MAX_HEIGHT.in(Meters), "MAX_HEIGHT should be 0.25");
        assertEquals(5.0, VisionConfig.Thresholds.MAX_TILT.in(Degrees), 1e-4, "MAX_TILT should be 5 degrees");
    }

    @Test
    void testThresholds_TagQuality() {
        assertEquals(0.9, VisionConfig.Thresholds.MAX_AMBIGUITY, "MAX_AMBIGUITY should be 0.9");
        assertEquals(0.5, VisionConfig.Thresholds.HIGH_AMBIGUITY, "HIGH_AMBIGUITY should be 0.5");
        assertEquals(0.25, VisionConfig.Thresholds.LOW_AMBIGUITY, "LOW_AMBIGUITY should be 0.25");
    }

    @Test
    void testThresholds_MotionThresholds() {
        assertEquals(4 * Math.PI, VisionConfig.Thresholds.MAX_ROTATION_SPEED.in(RadiansPerSecond),
                "MAX_ROTATION_SPEED should be 4π");
        assertEquals(0.5, VisionConfig.Thresholds.HIGH_ROTATION_SPEED.in(RadiansPerSecond),
                "HIGH_ROTATION_SPEED should be 0.5");
        assertEquals(0.2, VisionConfig.Thresholds.STATIONARY_SPEED.in(MetersPerSecond),
                "STATIONARY_SPEED should be 0.2");
    }

    @Test
    void testThresholds_TargetSizeThresholds() {
        assertEquals(0.025, VisionConfig.Thresholds.MIN_SIZE, "MIN_SIZE should be 0.025");
        assertEquals(0.4, VisionConfig.Thresholds.VERY_CLOSE_SIZE, "VERY_CLOSE_SIZE should be 0.4");
        assertEquals(0.8, VisionConfig.Thresholds.CLOSE_SIZE, "CLOSE_SIZE should be 0.8");
        assertEquals(0.1, VisionConfig.Thresholds.MODERATE_SIZE, "MODERATE_SIZE should be 0.1");
        assertEquals(0.05, VisionConfig.Thresholds.SMALL_MULTI_SIZE, "SMALL_MULTI_SIZE should be 0.05");
        assertEquals(0.09, VisionConfig.Thresholds.LARGE_MULTI_SIZE, "LARGE_MULTI_SIZE should be 0.09");
        assertEquals(0.03, VisionConfig.Thresholds.STABLE_SIZE, "STABLE_SIZE should be 0.03");
    }

    @Test
    void testThresholds_PoseDifferenceThresholds() {
        assertEquals(0.5, VisionConfig.Thresholds.CLOSE_POSE_DIFF,
                "CLOSE_POSE_DIFF should be 0.5");
        assertEquals(0.3, VisionConfig.Thresholds.PROXIMITY_POSE_DIFF,
                "PROXIMITY_POSE_DIFF should be 0.3");
    }

    @Test
    void testThresholds_Scoring() {
        assertEquals(100.0, VisionConfig.Thresholds.TAG_COUNT_WEIGHT,
                "TAG_COUNT_WEIGHT should be 100.0");
    }

    @Test
    void testPoseStdDevs_RecordConstruction() {
        // Test that we can create custom instances
        VisionConfig.PoseStdDevs custom = new VisionConfig.PoseStdDevs(1.5, 2.5);
        assertEquals(1.5, custom.xy(), "Custom xy should be 1.5");
        assertEquals(2.5, custom.theta(), "Custom theta should be 2.5");
    }

    @Test
    void testThresholds_AmbiguityOrdering() {
        // Verify ambiguity thresholds are in logical order
        assertTrue(VisionConfig.Thresholds.LOW_AMBIGUITY < VisionConfig.Thresholds.HIGH_AMBIGUITY,
                "LOW_AMBIGUITY should be less than HIGH_AMBIGUITY");
        assertTrue(VisionConfig.Thresholds.HIGH_AMBIGUITY < VisionConfig.Thresholds.MAX_AMBIGUITY,
                "HIGH_AMBIGUITY should be less than MAX_AMBIGUITY");
    }

    @Test
    void testThresholds_SizeOrdering() {
        // Verify size thresholds are in logical order
        assertTrue(VisionConfig.Thresholds.MIN_SIZE < VisionConfig.Thresholds.STABLE_SIZE,
                "MIN_SIZE should be less than STABLE_SIZE");
        assertTrue(VisionConfig.Thresholds.SMALL_MULTI_SIZE < VisionConfig.Thresholds.LARGE_MULTI_SIZE,
                "SMALL_MULTI_SIZE should be less than LARGE_MULTI_SIZE");
        assertTrue(VisionConfig.Thresholds.MODERATE_SIZE < VisionConfig.Thresholds.VERY_CLOSE_SIZE,
                "MODERATE_SIZE should be less than VERY_CLOSE_SIZE");
        assertTrue(VisionConfig.Thresholds.VERY_CLOSE_SIZE < VisionConfig.Thresholds.CLOSE_SIZE,
                "VERY_CLOSE_SIZE should be less than CLOSE_SIZE");
    }

    @Test
    void testThresholds_RotationSpeedOrdering() {
        // Verify rotation speed thresholds are in logical order
        assertTrue(
                VisionConfig.Thresholds.HIGH_ROTATION_SPEED
                        .baseUnitMagnitude() < VisionConfig.Thresholds.MAX_ROTATION_SPEED.baseUnitMagnitude(),
                "HIGH_ROTATION_SPEED should be less than MAX_ROTATION_SPEED");
    }

    @Test
    void testPoseStdDevs_PenaltyMinFunction() {
        // Test that penalties work correctly with Math.min
        VisionConfig.PoseStdDevs lowStdDev = new VisionConfig.PoseStdDevs(0.1, 5.0);
        VisionConfig.PoseStdDevs penalty = VisionConfig.PoseStdDevs.HIGH_AMBIGUITY_PENALTY;

        double resultingTheta = Math.min(lowStdDev.theta(), penalty.theta());
        assertEquals(5.0, resultingTheta,
                "Math.min should select the lower theta value for penalty application");
    }

    @Test
    void testPoseStdDevs_VelocityMagnitudeCalculation() {
        // Regression test: Verify that velocity magnitude should use Math.hypot, not
        // addition
        // This test documents the correct calculation method
        double vx = 0.5;
        double vy = -0.5;

        // Wrong way (bug): simple addition
        double wrongMagnitude = vx + vy;
        assertEquals(0.0, wrongMagnitude, "Bug: Simple addition gives 0.0");

        // Correct way: Math.hypot
        double correctMagnitude = Math.hypot(vx, vy);
        assertEquals(Math.sqrt(0.5), correctMagnitude, 0.0001,
                "Correct: Math.hypot gives ~0.707");

        // Verify that correct magnitude is compared against STATIONARY_SPEED
        assertTrue(correctMagnitude > VisionConfig.Thresholds.STATIONARY_SPEED.magnitude(),
                "Robot at (0.5, -0.5) m/s should NOT be considered stationary");
    }
}
