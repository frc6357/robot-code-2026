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
    private final Angle fixedLaunchAngle;                   // Fixed angle (for now)
    private final double wheelRadiusMeters;                 // Flywheel radius for velocity calculation
    private final double slipRatio;                         // Ball velocity / wheel velocity (1.0 = no slip)

    private final int maxIterations;
    private final Distance convergenceThreshold;
    private final Range validRange;

    // TODO: Future - add InterpolatingDoubleTreeMap launchAngleMap for adjustable angle

    private InterpolatedShotStrategy(
        InterpolatingDoubleTreeMap flywheelSpeedMap,
        Angle fixedLaunchAngle,
        double wheelRadiusMeters,
        double slipRatio,
        int maxIterations,
        Distance convergenceThreshold,
        Range validRange
    ) {
        this.flywheelSpeedMap = flywheelSpeedMap;
        this.fixedLaunchAngle = fixedLaunchAngle;
        this.wheelRadiusMeters = wheelRadiusMeters;
        this.slipRatio = slipRatio;
        this.maxIterations = maxIterations;
        this.convergenceThreshold = convergenceThreshold;
        this.validRange = validRange;
    }

    /**
     * Calculates time of flight using basic kinematics: t = d / (v * cos(θ))
     * 
     * @param distanceMeters Horizontal distance to target
     * @param flywheelRPM Flywheel speed in RPM
     * @return Time of flight in seconds
     */
    private double calculateTimeOfFlight(double distanceMeters, double flywheelRPM) {
        // Convert RPM to ball exit velocity: v = ω × r × slipRatio
        // slipRatio accounts for energy loss when ball contacts flywheel (1.0 = perfect grip)
        double flywheelRPS = flywheelRPM / 60.0;
        double exitVelocity = flywheelRPS * 2.0 * Math.PI * wheelRadiusMeters * slipRatio;
        
        // ToF = distance / (velocity × cos(launchAngle))
        double cosAngle = Math.cos(fixedLaunchAngle.in(Radians));
        
        // Prevent division by zero
        if (exitVelocity * cosAngle < 0.1) {
            return 1.0; // Fallback to 1 second
        }
        
        return distanceMeters / (exitVelocity * cosAngle);
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

        // Check if robot is effectively stationary (< 0.2 m/s in all directions)
        boolean isStationary = Math.abs(shooterVelocity.dx) < 0.2 
                            && Math.abs(shooterVelocity.dy) < 0.2 
                            && Math.abs(shooterVelocity.dz) < 0.2;

        if (isStationary) {
            // Skip iteration - robot is stationary, use current position
            converged = true;
            iterations = 0;
        } else {
            // Iterative lookahead: The robot moves while the projectile is in flight.
            // The ball inherits the robot's velocity when launched, so it will drift
            // in the direction the robot is moving.
            // 
            // To compensate: if robot moves RIGHT, ball drifts RIGHT, so aim LEFT.
            // We achieve this by projecting a "virtual target" in the OPPOSITE direction
            // of robot motion (or equivalently, shifting the shooter BACKWARD from its motion).
            // This makes the angle calculation yield an aim point that leads the motion.

            for (int i = 0; i < maxIterations; i++) {
                iterations = i + 1;

                // Get flywheel RPM for current distance, then calculate ToF kinematically
                double flywheelRPMForIteration = flywheelSpeedMap.get(distance);
                double tof = calculateTimeOfFlight(distance, flywheelRPMForIteration);

                // Calculate virtual shooter position OPPOSITE to velocity direction
                // If robot moves +X (right), virtual shooter is shifted LEFT (-X),
                // so angle from virtual shooter to target points more RIGHT → we aim RIGHT
                // Wait no... if virtual shooter is LEFT of current, angle to target is more RIGHT
                // But ball drifts RIGHT so we need to aim LEFT... 
                // Therefore: shift virtual shooter in SAME direction as velocity!
                // Then angle from shifted position to target is more LEFT → aim LEFT ✓
                //
                // NO WAIT - I keep confusing myself. Let me think physically:
                // - Robot at (3,3), target at (0,0), moving +X at 2m/s
                // - In 0.5s, robot will be at (4,3)
                // - Ball launched NOW needs to land at (0,0) in 0.5s
                // - Ball has initial +X velocity from robot
                // - Ball will drift right by 1m during flight → lands at (1, something)
                // - To hit (0,0), we need to aim LEFT so the drift brings it back
                // - Virtual target should be at (-1, 0) relative → aim more left
                //
                // So the virtual TARGET should shift OPPOSITE to robot velocity!
                // virtualTarget = target - (velocity * tof)
                // OR equivalently: shift shooter in +velocity direction and compute angle
                // 
                // Current code: futureShooterPose = shooterPose + velocity*tof (positive)
                // Angle from (4,3) to (0,0) = atan2(-3,-4) = 216.87° (more CW than 225°)
                // More CW = aim more RIGHT... but we need to aim LEFT!
                //
                // Fix: use NEGATIVE velocity shift
                Translation3d motionCompensation = new Translation3d(
                    -shooterVelocity.dx * tof,
                    -shooterVelocity.dy * tof,
                    -shooterVelocity.dz * tof
                );

                futureShooterPose = new Pose3d(
                    shooterPose.getTranslation().plus(motionCompensation),
                    shooterPose.getRotation()  // Keep current rotation for aiming reference
                );

                // Recalculate distance from shifted position to target
                double newDistance = futureShooterPose.getTranslation().getDistance(target);

                // Check convergence
                if (Math.abs(newDistance - distance) < convergenceThreshold.in(Meters)) {
                    converged = true;
                    distance = newDistance;
                    break;
                }

                distance = newDistance;
            }
        }

        // === STEP 2: Calculate launcher yaw to aim at target ===
        // futureShooterPose now represents current position with motion-compensated offset
        // The angle from this position to target gives us the correct lead angle
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

        // === STEP 4: Lookup flywheel RPM from map, calculate ToF kinematically ===
        double flywheelRPM = flywheelSpeedMap.get(distance);
        double tof = calculateTimeOfFlight(distance, flywheelRPM);

        // === STEP 5: Check shot validity ===
        boolean validShot = validRange.isValid(Meters.of(distance));

        return new ShotParameters(
            RPM.of(flywheelRPM),
            fixedLaunchAngle,  // Fixed for now
            Radians.of(launcherYaw.getRadians()),
            RadiansPerSecond.of(yawVelocity),
            target,
            new Translation3d(distance, new Rotation3d(launcherYaw)),
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
        private Angle fixedLaunchAngle = Degrees.of(55);
        private double wheelRadiusMeters = 0.0508; // Default: 2 inch radius
        private double slipRatio = 1.0; // 1.0 = no slip, < 1.0 = ball slower than wheel surface

        private int maxIterations = 10;
        private Distance convergenceThreshold = Centimeters.of(10);
        private Range validRange = new Range(Meters.of(1.0), Meters.of(5.0));

        public Builder withFlywheelSpeedMap(InterpolatingDoubleTreeMap map) {
            this.flywheelSpeedMap = map;
            return this;
        }

        public Builder withWheelRadius(double radiusMeters) {
            this.wheelRadiusMeters = radiusMeters;
            return this;
        }

        /**
         * Sets the slip ratio for ball velocity compensation.
         * @param ratio Ball velocity / wheel surface velocity (1.0 = perfect grip, 0.9 = 10% slip)
         */
        public Builder withSlipRatio(double ratio) {
            this.slipRatio = ratio;
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
                fixedLaunchAngle,
                wheelRadiusMeters,
                slipRatio,
                maxIterations,
                convergenceThreshold,
                validRange
            );
        }
    }
}
