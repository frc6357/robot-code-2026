package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Coordinator for shot calculations with motion compensation.
 *
 * <p>Handles:
 * <ul>
 *   <li>Phase delay compensation (predict where robot will be)</li>
 *   <li>Robot-to-shooter 3D transformation</li>
 *   <li>Delegation to calculation strategy</li>
 * </ul>
 *
 * <p>Usage:
 * <pre>{@code
 * ShotCalculator calc = new ShotCalculator.Builder()
 *     .withStrategy(strategy)
 *     .withRobotToShooterTransform(transform)
 *     .build();
 *
 * ShotParameters shot = calc.calculate(pose, velocity, target, timestamp);
 * }</pre>
 */
public class ShotCalculator {

    private final Transform3d robotToShooter;
    private final ShotCalculationStrategy strategy;
    private final double phaseDelay;

    private ShotCalculator(
        Transform3d robotToShooter,
        ShotCalculationStrategy strategy,
        double phaseDelay
    ) {
        this.robotToShooter = robotToShooter;
        this.strategy = strategy;
        this.phaseDelay = phaseDelay;
    }

    /**
     * Calculate shot parameters for a moving robot.
     *
     * @param robotPose Current robot pose (field-relative)
     * @param robotVelocity Current velocity (field-relative)
     * @param target Target position (2D field coordinates)
     * @param timestamp Current FPGA timestamp
     * @return Calculated shot parameters
     */
    public ShotParameters calculate(
        Pose2d robotPose,
        ChassisSpeeds robotVelocity,
        Translation2d target,
        double timestamp
    ) {
        // TODO: STUDENT EXERCISE - Phase delay compensation
        // The shot won't execute instantly - there's delay from code execution,
        // motor response, etc. Predict where the robot will be after phaseDelay.
        //
        // Steps:
        // 1. Create Twist2d: (vx * delay, vy * delay, omega * delay)
        // 2. Apply to pose: pose.exp(twist) returns future pose
        //
        // For now, just use current pose (no compensation):
        Pose2d compensatedPose = robotPose;

        // Transform robot pose to shooter pose (3D)
        Pose3d shooterPose = new Pose3d(compensatedPose).plus(robotToShooter);

        // Convert velocity to 3D twist
        Twist3d shooterVelocity = new Twist3d(
            robotVelocity.vxMetersPerSecond,
            robotVelocity.vyMetersPerSecond,
            0.0,  // No Z velocity
            0.0,  // No roll rate
            0.0,  // No pitch rate
            robotVelocity.omegaRadiansPerSecond
        );

        // Target at ground level (Z = 0)
        Translation3d target3d = new Translation3d(target.getX(), target.getY(), 0.0);

        // Delegate to strategy
        return strategy.calculate(
            shooterPose,
            shooterVelocity,
            target3d,
            CalculationContext.create(timestamp)
        );
    }

    /**
     * Calculate shot for stationary robot.
     */
    public ShotParameters calculateStationary(Pose2d robotPose, Translation2d target) {
        return calculate(robotPose, new ChassisSpeeds(), target, 0.0);
    }

    public ShotCalculationStrategy getStrategy() {
        return strategy;
    }

    public double getPhaseDelay() {
        return phaseDelay;
    }

    /**
     * Builder for ShotCalculator.
     */
    public static class Builder {
        private Transform3d robotToShooter = new Transform3d();
        private ShotCalculationStrategy strategy = null;
        private double phaseDelay = 0.02;  // 20ms default

        public Builder withRobotToShooterTransform(Transform3d transform) {
            this.robotToShooter = transform;
            return this;
        }

        public Builder withStrategy(ShotCalculationStrategy strategy) {
            this.strategy = strategy;
            return this;
        }

        public Builder withPhaseDelay(double seconds) {
            this.phaseDelay = seconds;
            return this;
        }

        public ShotCalculator build() {
            if (strategy == null) {
                throw new IllegalStateException("Strategy must be set");
            }
            return new ShotCalculator(robotToShooter, strategy, phaseDelay);
        }
    }
}
