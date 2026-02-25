package frc.robot.subsystems.launcher.moveandshoot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

/**
 * Iterative lookahead shot calculation strategy.
 *
 * <p>Calculates where to aim based on robot motion during projectile flight.
 * Currently supports fixed launch angle with controllable yaw and flywheel speed.
 *
 * <p>Algorithm:
 * <ol>
 *   <li>Calculate initial distance to target</li>
 *   <li>Iterate: project shooter position forward by time-of-flight, recalculate distance</li>
 *   <li>Repeat until distance converges (or max iterations)</li>
 *   <li>Calculate launcher yaw angle to aim at target</li>
 *   <li>Calculate yaw velocity feedforward for tracking</li>
 *   <li>Lookup flywheel RPM from interpolation map</li>
 * </ol>
 */
public final class InterpolatedShotStrategy implements ShotCalculationStrategy {

    private final InterpolatingDoubleTreeMap flywheelSpeedMap;  // distance → RPM
    private final InterpolatingDoubleTreeMap timeOfFlightMap;   // distance → seconds
    private final Angle fixedLaunchAngle;                   // Fixed angle (for now)

    private final int maxIterations;
    private final Distance convergenceThreshold;
    private final Range validRange;

    // TODO: Future - add InterpolatingDoubleTreeMap launchAngleMap for adjustable angle

    private InterpolatedShotStrategy(
        InterpolatingDoubleTreeMap flywheelSpeedMap,
        InterpolatingDoubleTreeMap timeOfFlightMap,
        Angle fixedLaunchAngle,
        int maxIterations,
        Distance convergenceThreshold,
        Range validRange
    ) {
        this.flywheelSpeedMap = flywheelSpeedMap;
        this.timeOfFlightMap = timeOfFlightMap;
        this.fixedLaunchAngle = fixedLaunchAngle;
        this.maxIterations = maxIterations;
        this.convergenceThreshold = convergenceThreshold;
        this.validRange = validRange;
    }

    @Override
    public ShotParameters calculate(
        Pose3d shooterPose,
        Twist3d shooterVelocity,
        Translation3d target,
        CalculationContext context
    ) {
        // === STEP 1: Iterative lookahead to find effective distance ===
        double distance = shooterPose.getTranslation().getDistance(target);
        Pose3d futureShooterPose = shooterPose;
        int iterations = 0;
        boolean converged = false;

        // Iterative lookahead: The robot moves while the projectile is in flight.
        // We aim where we WILL BE, not where we ARE. Each iteration refines the estimate:
        // 1. Get time-of-flight for current distance
        // 2. Project shooter position forward by ToF
        // 3. Recalculate distance from future position to target
        // 4. Repeat until distance converges

        for (int i = 0; i < maxIterations; i++) {
            iterations = i + 1;

            // Get time of flight for current distance estimate
            double tof = timeOfFlightMap.get(distance);

            // Project shooter position forward by ToF
            Translation3d linearMotion = new Translation3d(
                shooterVelocity.dx * tof,
                shooterVelocity.dy * tof,
                shooterVelocity.dz * tof
            );
            Rotation3d angularMotion = new Rotation3d(
                shooterVelocity.rx * tof,
                shooterVelocity.ry * tof,
                shooterVelocity.rz * tof
            );

            futureShooterPose = new Pose3d(
                shooterPose.getTranslation().plus(linearMotion),
                shooterPose.getRotation().plus(angularMotion)
            );

            // Recalculate distance from future position to target
            double newDistance = futureShooterPose.getTranslation().getDistance(target);

            // Check convergence
            if (Math.abs(newDistance - distance) < convergenceThreshold.in(Meters)) {
                converged = true;
                distance = newDistance;
                break;
            }

            distance = newDistance;
        }

        // === STEP 2: Calculate launcher yaw to aim at target ===
        Translation3d toTarget = target.minus(futureShooterPose.getTranslation());
        Rotation2d launcherYaw = new Rotation2d(toTarget.getX(), toTarget.getY());

        // === STEP 3: Calculate yaw velocity feedforward ===
        // As the robot moves, the angle to the target changes. Calculate the angular
        // velocity (rad/s) of the line-of-sight using ω = v_perpendicular / distance
        double yawVelocity = 0.0;
        if (distance > 0.1) {
            double lateralVelocity = Math.hypot(shooterVelocity.dx, shooterVelocity.dy);
            yawVelocity = lateralVelocity / distance;
        }

        // === STEP 4: Lookup flywheel RPM from map ===
        double flywheelRPM = flywheelSpeedMap.get(distance);
        double tof = timeOfFlightMap.get(distance);

        // === STEP 5: Check shot validity ===
        boolean validShot = validRange.isValid(Meters.of(distance));

        return new ShotParameters(
            RPM.of(flywheelRPM),
            fixedLaunchAngle,  // Fixed for now
            Radians.of(launcherYaw.getRadians()),
            RadiansPerSecond.of(yawVelocity),
            target,
            Meters.of(distance),
            Seconds.of(tof),
            getName(),
            iterations,
            converged,
            validShot,
            context.timestamp()
        );
    }

    @Override
    public Range getValidRange() {
        return validRange;
    }

    @Override
    public String getName() {
        return "Interpolated";
    }

    /**
     * Builder for InterpolatedShotStrategy.
     */
    public static class Builder {
        private InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
        private InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
        private Angle fixedLaunchAngle = Degrees.of(55);

        private int maxIterations = 10;
        private Distance convergenceThreshold = Centimeters.of(10);
        private Range validRange = new Range(Meters.of(1.0), Meters.of(5.0));

        public Builder withFlywheelSpeedMap(InterpolatingDoubleTreeMap map) {
            this.flywheelSpeedMap = map;
            return this;
        }

        public Builder withTimeOfFlightMap(InterpolatingDoubleTreeMap map) {
            this.timeOfFlightMap = map;
            return this;
        }

        public Builder withFixedLaunchAngle(Angle angle) {
            this.fixedLaunchAngle = angle;
            return this;
        }

        // TODO: Future - add withLaunchAngleMap() for adjustable angle

        public Builder withMaxIterations(int maxIterations) {
            this.maxIterations = maxIterations;
            return this;
        }

        public Builder withConvergenceThreshold(Distance distanceConvergenceThreshold) {
            this.convergenceThreshold = distanceConvergenceThreshold;
            return this;
        }

        public Builder withValidRange(Range range) {
            this.validRange = range;
            return this;
        }

        public InterpolatedShotStrategy build() {
            return new InterpolatedShotStrategy(
                flywheelSpeedMap,
                timeOfFlightMap,
                fixedLaunchAngle,
                maxIterations,
                convergenceThreshold,
                validRange
            );
        }
    }
}
