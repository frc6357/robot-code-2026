package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.vision.Limelight.LimelightConfig;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Verifies that the turret Limelight camera pose math is correct end-to-end:
 *
 *  1. LimelightConfig.withTranslation/withRotation stores cameraPose3d in WPILib
 *     convention (X=forward, Y=left, Z=up; pitch positive = nose DOWN).
 *
 *  2. Pose3d.rotateAround() (WPILib math, Y-left frame) correctly sweeps the
 *     camera position around the turret pivot as the turret angle changes.
 *
 *  3. LimelightConfig stores the raw LL values (right-positive, pitch-positive=up)
 *     unchanged so FuelDetection and the no-arg setCameraPoseInRobotSpace() still work.
 *
 * Physical setup (from Konstants):
 *   Turret LL at 0 deg (facing LEFT / +Y in WPILib):
 *     forward  = -8.0"   -> X = -0.2032 m
 *     right    = -13.25" -> Y = +13.25" left = +0.3366 m
 *     up       = 17.833" -> Z = +0.4530 m
 *     roll=0 deg, pitch=9 deg (nose up in LL -> nose DOWN in WPILib -> -9 deg), yaw=90 deg (faces left)
 *
 *   Turret pivot (WPILib convention):
 *     X = -8.0"  = -0.2032 m
 *     Y = +6.532" = +0.1659 m  (left of center)
 *     Z = 17.833" = +0.4530 m
 *
 * Turret angle convention: 0 deg = left, CCW positive.
 *   Turret   0 deg -> camera faces left  (+Y WPILib) -> WPILib yaw = +90 deg
 *   Turret +90 deg -> camera faces back  (-X WPILib) -> WPILib yaw = 180 deg
 *   Turret -90 deg -> camera faces front (+X WPILib) -> WPILib yaw =   0 deg
 */
class TurretCameraPoseTest {

    // Tolerances
    private static final double TRANSLATION_TOL_M = 1e-4; // 0.1 mm
    private static final double ANGLE_TOL_DEG     = 1e-3; // 0.001 deg

    // -------------------------------------------------------------------------
    // Constants mirrored from Konstants.java
    // -------------------------------------------------------------------------

    private static final double LL_FORWARD_M = Units.inchesToMeters(-8.0);
    private static final double LL_RIGHT_M   = Units.inchesToMeters(-13.25); // LL right (negative = left)
    private static final double LL_UP_M      = Units.inchesToMeters(17.833);
    private static final double LL_ROLL_DEG  = 0.0;
    private static final double LL_PITCH_DEG = 9.0;  // LL convention: nose up
    private static final double LL_YAW_DEG   = 90.0; // LL convention: 90 deg CCW = faces left

    private static final Translation3d PIVOT = new Translation3d(
        Units.inchesToMeters(-8.0),   // forward
        Units.inchesToMeters(6.532),  // left (WPILib Y+)
        Units.inchesToMeters(17.833)  // up
    );

    /** Builds the turret LL config, matching VisionConfig.TURRET_CONFIG. */
    private LimelightConfig buildTurretConfig() {
        return new LimelightConfig("limelight-turret")
            .withTranslation(LL_FORWARD_M, LL_RIGHT_M, LL_UP_M)
            .withRotation(LL_ROLL_DEG, LL_PITCH_DEG, LL_YAW_DEG);
    }

    /**
     * Applies the same rotation SKVision.periodic() does:
     *   Rotation3d(0, 0, turretAngleDeg in radians)
     * and returns the resulting WPILib-convention Pose3d.
     */
    private Pose3d applyTurretRotation(LimelightConfig config, double turretAngleDeg) {
        Rotation3d turretRotation = new Rotation3d(0, 0, Math.toRadians(turretAngleDeg));
        return config.getCameraPose3d().rotateAround(PIVOT, turretRotation);
    }

    // =========================================================================
    // 1. LimelightConfig stores cameraPose3d in WPILib convention at turret 0 deg
    // =========================================================================

    @Test
    void configStoresWPILibConvention_Translation() {
        LimelightConfig cfg = buildTurretConfig();
        Pose3d pose = cfg.getCameraPose3d();

        // X = forward (same sign as LL forward)
        assertEquals(LL_FORWARD_M, pose.getTranslation().getX(), TRANSLATION_TOL_M,
            "X (forward) must equal LL forward");

        // Y = LEFT-positive (WPILib), so Y = -right = -(-13.25") = +13.25" = +0.3366 m
        assertEquals(-LL_RIGHT_M, pose.getTranslation().getY(), TRANSLATION_TOL_M,
            "Y (left) must equal -right: LL right=-13.25\" -> WPILib Y=+13.25\"");

        // Z = up (same sign)
        assertEquals(LL_UP_M, pose.getTranslation().getZ(), TRANSLATION_TOL_M,
            "Z (up) must equal LL up");
    }

    @Test
    void configStoresWPILibConvention_Rotation() {
        LimelightConfig cfg = buildTurretConfig();
        Pose3d pose = cfg.getCameraPose3d();

        // Roll: same sign between LL and WPILib
        assertEquals(Math.toRadians(LL_ROLL_DEG), pose.getRotation().getX(), 1e-9,
            "Roll must be stored as-is");

        // Pitch: LL positive = nose UP; WPILib positive = nose DOWN -> negated in storage
        assertEquals(Math.toRadians(-LL_PITCH_DEG), pose.getRotation().getY(), 1e-9,
            "Pitch must be negated: LL 9 deg nose-up -> WPILib -9 deg (nose-down is +)");

        // Yaw: same sign (both CCW-positive)
        assertEquals(Math.toRadians(LL_YAW_DEG), pose.getRotation().getZ(), 1e-9,
            "Yaw must be stored as-is (both CCW-positive)");
    }

    @Test
    void rawConfigFieldsUnchanged() {
        // The raw LL-convention fields must NOT be touched, since FuelDetection uses them.
        LimelightConfig cfg = buildTurretConfig();

        assertEquals(LL_FORWARD_M, cfg.getForward(), TRANSLATION_TOL_M);
        assertEquals(LL_RIGHT_M,   cfg.getRight(),   TRANSLATION_TOL_M);
        assertEquals(LL_UP_M,      cfg.getUp(),      TRANSLATION_TOL_M);
        assertEquals(LL_ROLL_DEG,  cfg.getRoll(),    ANGLE_TOL_DEG);
        assertEquals(LL_PITCH_DEG, cfg.getPitch(),   ANGLE_TOL_DEG);
        assertEquals(LL_YAW_DEG,   cfg.getYaw(),     ANGLE_TOL_DEG);
    }

    // =========================================================================
    // 2. Turret at 0 deg -- camera pose should be unchanged from config
    //    (identity rotation, no displacement)
    // =========================================================================

    @Test
    void turretAt0Deg_poseUnchanged() {
        // Rotating by 0 deg must return the exact same pose.
        LimelightConfig cfg = buildTurretConfig();
        Pose3d original = cfg.getCameraPose3d();
        Pose3d rotated  = applyTurretRotation(cfg, 0.0);

        assertTranslationEquals(original.getTranslation(), rotated.getTranslation(), "Turret 0 deg: translation");
        assertRotationEquals(original.getRotation(), rotated.getRotation(), "Turret 0 deg: rotation");
    }

    // =========================================================================
    // 3. Turret at +90 deg -- camera faces BACKWARD (yaw 180 deg)
    //
    //  At 0 deg the camera is at (X=-0.2032, Y=+0.3366, Z=+0.4530) facing left (yaw=90).
    //  Pivot is at               (X=-0.2032, Y=+0.1659, Z=+0.4530).
    //
    //  Camera offset from pivot in XY:
    //    dx = -0.2032 - (-0.2032) = 0
    //    dy = +0.3366 - +0.1659   = +0.1707  (13.25" - 6.532" = 6.718" left of pivot)
    //
    //  Rotating +90 deg CCW (WPILib +Z rotation) in XY:
    //    new_dx = cos(90)*dx - sin(90)*dy =  0 - 0.1707 = -0.1707  (backward)
    //    new_dy = sin(90)*dx + cos(90)*dy =  0 + 0      =  0.000
    //
    //  New camera position in robot space:
    //    X = -0.2032 + (-0.1707) = -0.3739
    //    Y = +0.1659 + 0          = +0.1659
    //    Z = unchanged             = +0.4530
    //
    //  New yaw: 90 + 90 = 180 deg (camera faces backward)
    //
    //  At -90 deg turret angle:
    //    new_dx = +0.1707 (forward), yaw = 90 - 90 = 0 deg (camera faces forward)
    // =========================================================================

    @Test
    void turretAtPlus90Deg_cameraFacesBackward() {
        LimelightConfig cfg = buildTurretConfig();
        Pose3d rotated = applyTurretRotation(cfg, 90.0);

        // After +90 deg CCW rotation: camera yaw 90 deg (left) -> 180 deg (backward)
        double yawDeg = Math.toDegrees(rotated.getRotation().getZ());
        assertEquals(180.0, normalizeAngle(yawDeg), ANGLE_TOL_DEG,
            "Turret +90 deg CCW: camera (originally facing left at yaw=90) should now face backward (yaw=180)");
    }

    @Test
    void turretAtMinus90Deg_cameraFacesForward() {
        LimelightConfig cfg = buildTurretConfig();
        Pose3d rotated = applyTurretRotation(cfg, -90.0);

        // After -90 deg CW rotation: camera yaw 90 deg (left) -> 0 deg (forward)
        double yawDeg = Math.toDegrees(rotated.getRotation().getZ());
        assertEquals(0.0, normalizeAngle(yawDeg), ANGLE_TOL_DEG,
            "Turret -90 deg CW: camera (originally facing left at yaw=90) should now face forward (yaw=0)");
    }

    // =========================================================================
    // 4. Translation correctness at +90 deg turret angle
    //
    //  Camera arm from pivot: 6.718" = 13.25" - 6.532"
    //
    //  After +90 deg CCW rotation:
    //    X = pivot.X - arm = -8.0" - 6.718" = -14.718" = -0.3739 m  (moved backward)
    //    Y = pivot.Y       = +6.532"         = +0.1659 m  (unchanged)
    //    Z = unchanged     = 17.833"         = +0.4530 m
    //
    //  After -90 deg CW rotation:
    //    X = pivot.X + arm = -8.0" + 6.718" = -1.282" = -0.0326 m  (moved forward)
    //    Y = pivot.Y       = +6.532"                               (unchanged)
    //    Z = unchanged
    // =========================================================================

    @Test
    void turretAtPlus90Deg_translationCorrect() {
        LimelightConfig cfg = buildTurretConfig();
        Pose3d rotated = applyTurretRotation(cfg, 90.0);

        double armM = Units.inchesToMeters(6.718); // arm from pivot to camera

        double expectedX = Units.inchesToMeters(-8.0) - armM; // pivot.X - arm (moved backward)
        double expectedY = Units.inchesToMeters(6.532);       // pivot.Y unchanged
        double expectedZ = Units.inchesToMeters(17.833);      // Z unchanged

        assertEquals(expectedX, rotated.getTranslation().getX(), TRANSLATION_TOL_M,
            "Turret +90 deg: X should move backward by arm length (6.718\")");
        assertEquals(expectedY, rotated.getTranslation().getY(), TRANSLATION_TOL_M,
            "Turret +90 deg: Y should equal pivot Y (arm swept to purely X direction)");
        assertEquals(expectedZ, rotated.getTranslation().getZ(), TRANSLATION_TOL_M,
            "Turret +90 deg: Z (up) must not change");
    }

    @Test
    void turretAtMinus90Deg_translationCorrect() {
        LimelightConfig cfg = buildTurretConfig();
        Pose3d rotated = applyTurretRotation(cfg, -90.0);

        double armM = Units.inchesToMeters(6.718);

        double expectedX = Units.inchesToMeters(-8.0) + armM; // pivot.X + arm (moved forward)
        double expectedY = Units.inchesToMeters(6.532);       // pivot.Y unchanged
        double expectedZ = Units.inchesToMeters(17.833);      // Z unchanged

        assertEquals(expectedX, rotated.getTranslation().getX(), TRANSLATION_TOL_M,
            "Turret -90 deg: X should move forward by arm length (6.718\")");
        assertEquals(expectedY, rotated.getTranslation().getY(), TRANSLATION_TOL_M,
            "Turret -90 deg: Y should equal pivot Y");
        assertEquals(expectedZ, rotated.getTranslation().getZ(), TRANSLATION_TOL_M,
            "Turret -90 deg: Z must not change");
    }

    // =========================================================================
    // 5. Turret at +180 deg -- camera faces rightward (yaw = 90 + 180 = 270 = -90 deg)
    //    i.e. faces the RIGHT side of the robot.
    //
    //  Camera arm rotated 180 deg:
    //    new_dx = -0 = 0
    //    new_dy = -arm = -0.1707
    //
    //  New position:
    //    X = -0.2032 + 0       = -0.2032  (same forward as pivot)
    //    Y = +0.1659 - 0.1707  = -0.0048  (slightly right of center)
    //    Z = +0.4530
    // =========================================================================

    @Test
    void turretAt180Deg_cameraFacesRight() {
        LimelightConfig cfg = buildTurretConfig();
        Pose3d rotated = applyTurretRotation(cfg, 180.0);

        // Yaw: 90 + 180 = 270 deg -> normalize to -90 deg
        double yawDeg = Math.toDegrees(rotated.getRotation().getZ());
        assertEquals(-90.0, normalizeAngle(yawDeg), ANGLE_TOL_DEG,
            "Turret +180 deg: camera should face right (yaw = -90 deg)");
    }

    @Test
    void turretAt180Deg_translationCorrect() {
        LimelightConfig cfg = buildTurretConfig();
        Pose3d rotated = applyTurretRotation(cfg, 180.0);

        double armM = Units.inchesToMeters(6.718);

        double expectedX = Units.inchesToMeters(-8.0);         // X unchanged (arm was perpendicular to X)
        double expectedY = Units.inchesToMeters(6.532) - armM; // pivot.Y - arm (arm now points right/-Y)
        double expectedZ = Units.inchesToMeters(17.833);

        assertEquals(expectedX, rotated.getTranslation().getX(), TRANSLATION_TOL_M,
            "Turret +180 deg: X should be the same as pivot X");
        assertEquals(expectedY, rotated.getTranslation().getY(), TRANSLATION_TOL_M,
            "Turret +180 deg: Y = pivot.Y - arm (arm now points right/-Y)");
        assertEquals(expectedZ, rotated.getTranslation().getZ(), TRANSLATION_TOL_M,
            "Turret +180 deg: Z must not change");
    }

    // =========================================================================
    // 6. Z (up) is invariant across all turret angles
    //    Rotating around a vertical (Z) axis must never change Z.
    // =========================================================================

    @Test
    void upIsAlwaysPreserved_variousAngles() {
        LimelightConfig cfg = buildTurretConfig();
        double[] angles = { 0, 30, 45, 60, 90, 120, 135, 150, 180, -30, -90, -120, -180 };

        for (double angle : angles) {
            Pose3d rotated = applyTurretRotation(cfg, angle);
            assertEquals(LL_UP_M, rotated.getTranslation().getZ(), TRANSLATION_TOL_M,
                "Z (up) must be invariant at turret angle " + angle + " deg");
        }
    }

    // =========================================================================
    // 7. Pitch is preserved through rotation (only yaw and XY change)
    //    A pure Z-axis rotation must not alter pitch.
    // =========================================================================

    @Test
    void pitchIsPreservedThroughRotation() {
        LimelightConfig cfg = buildTurretConfig();
        double originalPitchRad = cfg.getCameraPose3d().getRotation().getY();
        double[] angles = { 0, 45, 90, -90, 180 };

        for (double angle : angles) {
            Pose3d rotated = applyTurretRotation(cfg, angle);
            assertEquals(originalPitchRad, rotated.getRotation().getY(), 1e-9,
                "Pitch (Y rotation) must be unchanged at turret angle " + angle + " deg");
        }
    }

    // =========================================================================
    // 8. setCameraPoseInRobotSpace(Pose3d) WPILib->LL conversion
    //    Verify that the Pose3d overload correctly converts signs before
    //    sending to LimelightHelpers. We do this by checking the Pose3d
    //    at turret 0 deg converts back to the exact original LL values.
    //    (We can't call the real LL method in unit tests since it writes
    //     to NetworkTables, but we can verify the math directly.)
    // =========================================================================

    @Test
    void pose3dToLLConversion_at0Deg() {
        LimelightConfig cfg = buildTurretConfig();
        Pose3d wpiPose = cfg.getCameraPose3d();

        // Manually apply the same conversions as setCameraPoseInRobotSpace(Pose3d)
        double llForward = wpiPose.getTranslation().getX();
        double llSide    = -wpiPose.getTranslation().getY(); // WPILib left -> LL right (negate)
        double llUp      = wpiPose.getTranslation().getZ();
        double llRoll    = Math.toDegrees(wpiPose.getRotation().getX());
        double llPitch   = -Math.toDegrees(wpiPose.getRotation().getY()); // WPILib nose-down -> LL nose-up (negate)
        double llYaw     = Math.toDegrees(wpiPose.getRotation().getZ());

        // These must round-trip back to the original LL config values
        assertEquals(LL_FORWARD_M, llForward, TRANSLATION_TOL_M, "Forward round-trip");
        assertEquals(LL_RIGHT_M,   llSide,    TRANSLATION_TOL_M, "Side (right) round-trip");
        assertEquals(LL_UP_M,      llUp,      TRANSLATION_TOL_M, "Up round-trip");
        assertEquals(LL_ROLL_DEG,  llRoll,    ANGLE_TOL_DEG,     "Roll round-trip");
        assertEquals(LL_PITCH_DEG, llPitch,   ANGLE_TOL_DEG,     "Pitch round-trip");
        assertEquals(LL_YAW_DEG,   llYaw,     ANGLE_TOL_DEG,     "Yaw round-trip");
    }

    @Test
    void pose3dToLLConversion_atPlus90Deg() {
        // At turret +90 deg, camera faces backward. The LL values sent should reflect that.
        LimelightConfig cfg = buildTurretConfig();
        Pose3d wpiPose = applyTurretRotation(cfg, 90.0);

        double llYaw = Math.toDegrees(wpiPose.getRotation().getZ());
        // WPILib yaw after rotation: 90 + 90 = 180 deg
        // LL yaw is same sign, so LL should receive 180 deg (facing backward)
        assertEquals(180.0, normalizeAngle(llYaw), ANGLE_TOL_DEG,
            "LL yaw at turret +90 deg should be 180 deg (backward)");

        double llSide = -wpiPose.getTranslation().getY();
        // At turret +90 deg: Y = pivot.Y = +6.532" left -> LL side = -6.532" (negative = left)
        assertEquals(Units.inchesToMeters(-6.532), llSide, TRANSLATION_TOL_M,
            "LL side (right) at turret +90 deg: camera is at pivot lateral position, which is 6.532\" left");
    }

    // =========================================================================
    // Helper methods
    // =========================================================================

    /** Normalizes an angle to (-180, +180] degrees. */
    private static double normalizeAngle(double deg) {
        deg = deg % 360.0;
        if (deg > 180.0)   deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
    }

    private static void assertTranslationEquals(
            Translation3d expected, Translation3d actual, String msg) {
        assertEquals(expected.getX(), actual.getX(), TRANSLATION_TOL_M, msg + " X");
        assertEquals(expected.getY(), actual.getY(), TRANSLATION_TOL_M, msg + " Y");
        assertEquals(expected.getZ(), actual.getZ(), TRANSLATION_TOL_M, msg + " Z");
    }

    private static void assertRotationEquals(
            Rotation3d expected, Rotation3d actual, String msg) {
        assertEquals(expected.getX(), actual.getX(), 1e-9, msg + " roll");
        assertEquals(expected.getY(), actual.getY(), 1e-9, msg + " pitch");
        assertEquals(expected.getZ(), actual.getZ(), 1e-9, msg + " yaw");
    }
}
