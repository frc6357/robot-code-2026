package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.vision.Limelight;
import frc.lib.vision.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.drive.SKSwerve;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.Optional;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

/**
 * Unit tests for SKVision refactored methods.
 * Tests the actual logic and bug fixes, not just constant values.
 */
class SKVisionTest {

    private SKVision vision;
    private Limelight mockLL;

    @BeforeEach
    void setUp() {
        SKSwerve mockSwerve = mock(SKSwerve.class);
        mockLL = mock(Limelight.class);
        vision = new SKVision(Optional.of(mockSwerve));
    }

    // ============================================================
    // findHighestAmbiguity() Tests
    // ============================================================

    @Test
    void testFindHighestAmbiguity_SingleTag() {
        RawFiducial tag = new RawFiducial(1, 0, 0, 0, 0, 0, 0.5);
        when(mockLL.getTagStatus()).thenReturn("");

        double result = vision.findHighestAmbiguity(new RawFiducial[] { tag }, mockLL);

        assertEquals(0.5, result, "Should return the single tag's ambiguity");
        verify(mockLL).setTagStatus("");
        verify(mockLL).setTagStatus("Tag 1: 0.5");
    }

    @Test
    void testFindHighestAmbiguity_MultipleTags() {
        RawFiducial tag1 = new RawFiducial(1, 0, 0, 0, 0, 0, 0.3);
        RawFiducial tag2 = new RawFiducial(2, 0, 0, 0, 0, 0, 0.7);
        RawFiducial tag3 = new RawFiducial(3, 0, 0, 0, 0, 0, 0.5);

        double result = vision.findHighestAmbiguity(new RawFiducial[] { tag1, tag2, tag3 }, mockLL);

        assertEquals(0.7, result, "Should return the highest ambiguity value");
    }

    @Test
    void testFindHighestAmbiguity_EmptyArray() {
        double result = vision.findHighestAmbiguity(new RawFiducial[] {}, mockLL);

        assertEquals(-1, result, "Should return -1 for empty array (not 2, which was the bug)");
    }

    @Test
    void testFindHighestAmbiguity_RegressionTest_InitialValue() {
        // Regression test: The bug was initializing to 2 instead of -1
        // If all tags have ambiguity < 2, the old code would return 2
        // The fixed code should return the actual highest value
        RawFiducial tag1 = new RawFiducial(1, 0, 0, 0, 0, 0, 0.1);
        RawFiducial tag2 = new RawFiducial(2, 0, 0, 0, 0, 0, 0.2);

        double result = vision.findHighestAmbiguity(new RawFiducial[] { tag1, tag2 }, mockLL);

        assertEquals(0.2, result, "Fixed code should return 0.2, not 2 (the old bug)");
        assertNotEquals(2.0, result, "Should NOT return 2 (the bug value)");
    }

    // ============================================================
    // shouldRejectPose() Tests
    // ============================================================

    @Test
    void testShouldRejectPose_HighAmbiguity() {
        SKVision.VisionMeasurement m = createValidMeasurement();
        // Override highestAmbiguity to be too high
        m = new SKVision.VisionMeasurement(
                m.timeStamp(), m.targetSize(), m.botpose3D(), m.botposeMT1(), m.botposeMT2(),
                m.tags(), m.multiTags(), m.robotSpeed(), m.poseDifference(),
                0.95 // > MAX_AMBIGUITY (0.9)
        );

        boolean result = vision.shouldRejectPose(m, mockLL);

        assertTrue(result, "Should reject pose with ambiguity > 0.9");
        verify(mockLL).sendInvalidStatus("ambiguity rejection");
    }

    @Test
    void testShouldRejectPose_ExcessiveRotation() {
        SKVision.VisionMeasurement m = createValidMeasurement();
        // Override robot speed to have excessive rotation
        ChassisSpeeds fastRotation = new ChassisSpeeds(0, 0, 13.0); // > 4π (12.566)
        m = new SKVision.VisionMeasurement(
                m.timeStamp(), m.targetSize(), m.botpose3D(), m.botposeMT1(), m.botposeMT2(),
                m.tags(), m.multiTags(), fastRotation, m.poseDifference(), m.highestAmbiguity());

        boolean result = vision.shouldRejectPose(m, mockLL);

        assertTrue(result, "Should reject pose with rotation >= 4π rad/s");
        verify(mockLL).sendInvalidStatus("rot speed rejection - too fast");
    }

    @Test
    void testShouldRejectPose_ExcessiveHeight() {
        SKVision.VisionMeasurement m = createValidMeasurement();
        // Override pose to be floating in air
        Pose3d floatingPose = new Pose3d(1.0, 1.0, 0.3, new Rotation3d()); // z > 0.25
        m = new SKVision.VisionMeasurement(
                m.timeStamp(), m.targetSize(), floatingPose, m.botposeMT1(), m.botposeMT2(),
                m.tags(), m.multiTags(), m.robotSpeed(), m.poseDifference(), m.highestAmbiguity());

        boolean result = vision.shouldRejectPose(m, mockLL);

        assertTrue(result, "Should reject pose with height > 0.25m");
        verify(mockLL).sendInvalidStatus("height rejection - in air");
    }

    @Test
    void testShouldRejectPose_TargetTooSmall() {
        SKVision.VisionMeasurement m = createValidMeasurement();
        // Override target size to be too small
        m = new SKVision.VisionMeasurement(
                m.timeStamp(), 0.02, // <= MIN_SIZE (0.025)
                m.botpose3D(), m.botposeMT1(), m.botposeMT2(),
                m.tags(), m.multiTags(), m.robotSpeed(), m.poseDifference(), m.highestAmbiguity());

        boolean result = vision.shouldRejectPose(m, mockLL);

        assertTrue(result, "Should reject pose with target size <= 0.025");
        verify(mockLL).sendInvalidStatus("size rejection - target too small");
    }

    @Test
    void testShouldRejectPose_ValidPose() {
        SKVision.VisionMeasurement m = createValidMeasurement();

        boolean result = vision.shouldRejectPose(m, mockLL);

        assertFalse(result, "Should NOT reject a valid pose");
        verify(mockLL, never()).sendInvalidStatus(anyString());
    }

    // ============================================================
    // calculateIntegrationStdDevs() Tests - Including Velocity Bug Fix
    // ============================================================

    @Test
    void testCalculateIntegrationStdDevs_StationaryClose() {
        ChassisSpeeds stationary = new ChassisSpeeds(0.1, 0.1, 0);
        double largeTarget = 0.5; // > VERY_CLOSE_SIZE (0.4)

        VisionConfig.PoseStdDevs result = vision.calculateIntegrationStdDevs(
                stationary, largeTarget, false, 0.3, 0.4, mockLL);

        assertNotNull(result);
        assertEquals(VisionConfig.PoseStdDevs.STATIONARY_CLOSE, result);
        verify(mockLL).sendValidStatus("Stationary close integration");
    }

    @Test
    void testCalculateIntegrationStdDevs_VelocityMagnitudeBugFix() {
        // REGRESSION TEST for the velocity magnitude bug
        // Old bug: vx + vy (0.5 + -0.5 = 0, considered stationary)
        // Fixed: Math.hypot(0.5, -0.5) = ~0.707, NOT stationary
        ChassisSpeeds movingDiagonally = new ChassisSpeeds(0.5, -0.5, 0);
        double largeTarget = 0.5; // > VERY_CLOSE_SIZE (0.4)

        // Calculate actual magnitude
        double actualMagnitude = Math.hypot(0.5, -0.5);
        assertTrue(actualMagnitude > VisionConfig.Thresholds.STATIONARY_SPEED.magnitude(),
                "Robot moving at (0.5, -0.5) should NOT be considered stationary");

        VisionConfig.PoseStdDevs result = vision.calculateIntegrationStdDevs(
                movingDiagonally, largeTarget, false, 0.3, 0.4, mockLL);

        // Should NOT get STATIONARY_CLOSE because robot is moving
        assertNotEquals(VisionConfig.PoseStdDevs.STATIONARY_CLOSE, result,
                "Fixed code should NOT consider (0.5, -0.5) m/s as stationary");

        // With the bug, it would have returned STATIONARY_CLOSE
        // With the fix, it should use a different integration strategy
    }

    @Test
    void testCalculateIntegrationStdDevs_MultiTag() {
        ChassisSpeeds moving = new ChassisSpeeds(1.0, 0, 0);
        double mediumTarget = 0.07; // > SMALL_MULTI_SIZE (0.05), < LARGE_MULTI_SIZE (0.09)

        VisionConfig.PoseStdDevs result = vision.calculateIntegrationStdDevs(
                moving, mediumTarget, true, 0.3, 0.4, mockLL);

        assertNotNull(result);
        assertEquals(VisionConfig.PoseStdDevs.MULTI_TAG, result);
        verify(mockLL).sendValidStatus("Multi integration");
    }

    @Test
    void testCalculateIntegrationStdDevs_StrongMulti() {
        ChassisSpeeds moving = new ChassisSpeeds(1.0, 0, 0);
        double largeMultiTarget = 0.10; // > LARGE_MULTI_SIZE (0.09)

        VisionConfig.PoseStdDevs result = vision.calculateIntegrationStdDevs(
                moving, largeMultiTarget, true, 0.3, 0.4, mockLL);

        assertNotNull(result);
        assertEquals(VisionConfig.PoseStdDevs.STRONG_MULTI, result);
        verify(mockLL).sendValidStatus("Strong Multi integration");
    }

    @Test
    void testCalculateIntegrationStdDevs_CloseSingle() {
        ChassisSpeeds moving = new ChassisSpeeds(1.0, 0, 0);
        double veryLargeTarget = 0.85; // > CLOSE_SIZE (0.8)
        double closePose = 0.4; // < CLOSE_POSE_DIFF (0.5)

        VisionConfig.PoseStdDevs result = vision.calculateIntegrationStdDevs(
                moving, veryLargeTarget, false, 0.3, closePose, mockLL);

        assertNotNull(result);
        assertEquals(VisionConfig.PoseStdDevs.CLOSE_SINGLE, result);
        verify(mockLL).sendValidStatus("Close integration");
    }

    @Test
    void testCalculateIntegrationStdDevs_Proximity() {
        ChassisSpeeds moving = new ChassisSpeeds(1.0, 0, 0);
        double moderateTarget = 0.15; // > MODERATE_SIZE (0.1)
        double proximityPose = 0.25; // < PROXIMITY_POSE_DIFF (0.3)

        VisionConfig.PoseStdDevs result = vision.calculateIntegrationStdDevs(
                moving, moderateTarget, false, 0.3, proximityPose, mockLL);

        assertNotNull(result);
        assertEquals(VisionConfig.PoseStdDevs.PROXIMITY, result);
        verify(mockLL).sendValidStatus("Proximity integration");
    }

    @Test
    void testCalculateIntegrationStdDevs_Stable() {
        ChassisSpeeds moving = new ChassisSpeeds(1.0, 0, 0);
        double smallTarget = 0.04; // >= STABLE_SIZE (0.03)
        double lowAmbiguity = 0.2; // < LOW_AMBIGUITY (0.25)

        VisionConfig.PoseStdDevs result = vision.calculateIntegrationStdDevs(
                moving, smallTarget, false, lowAmbiguity, 0.5, mockLL);

        assertNotNull(result);
        assertEquals(VisionConfig.PoseStdDevs.STABLE, result);
        verify(mockLL).sendValidStatus("Stable integration");
    }

    @Test
    void testCalculateIntegrationStdDevs_Rejection() {
        ChassisSpeeds moving = new ChassisSpeeds(1.0, 0, 0);
        double tinyTarget = 0.03; // Doesn't meet any integration criteria
        double highAmbiguity = 0.5; // Not low enough for stable
        double largePoseDiff = 1.0; // Too large

        VisionConfig.PoseStdDevs result = vision.calculateIntegrationStdDevs(
                moving, tinyTarget, false, highAmbiguity, largePoseDiff, mockLL);

        assertNull(result, "Should reject when no integration criteria are met");
        verify(mockLL).sendInvalidStatus(contains("catch rejection"));
    }

    // ============================================================
    // Helper Methods
    // ============================================================

    private SKVision.VisionMeasurement createValidMeasurement() {
        Pose3d botpose3D = new Pose3d(1.0, 1.0, 0.1, new Rotation3d());
        Pose2d botposeMT1 = botpose3D.toPose2d();
        Pose2d botposeMT2 = new Pose2d(new Translation2d(1.0, 1.0), new Rotation2d());
        RawFiducial[] tags = new RawFiducial[0];
        ChassisSpeeds robotSpeed = new ChassisSpeeds(0.5, 0.5, 0.1);

        return new SKVision.VisionMeasurement(
                0.0, // timeStamp
                0.5, // targetSize
                botpose3D,
                botposeMT1,
                botposeMT2,
                tags,
                false, // multiTags
                robotSpeed,
                0.2, // poseDifference
                0.3 // highestAmbiguity
        );
    }
}
