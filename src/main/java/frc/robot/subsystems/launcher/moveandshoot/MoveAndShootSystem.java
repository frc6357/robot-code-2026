package frc.robot.subsystems.launcher.moveandshoot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Konstants.LauncherConstants.kConvergenceThresholdMeters;
import static frc.robot.Konstants.LauncherConstants.kFixedLaunchAngle;
import static frc.robot.Konstants.LauncherConstants.kMaxIterations;
import static frc.robot.Konstants.LauncherConstants.kMaxRangeMeters;
import static frc.robot.Konstants.LauncherConstants.kMinRangeMeters;

import frc.robot.Konstants.LauncherConstants;
import frc.robot.subsystems.launcher.mechanisms.BangBangLauncher;
import frc.robot.subsystems.launcher.moveandshoot.ShotCalculationStrategy.Range;
import frc.robot.subsystems.turret.SK26Turret;

public class MoveAndShootSystem {
    BangBangLauncher launcher;
    SK26Turret turret;

    ShotCalculator shotCalculator = createShotCalculator();

    ShootingCoordinator coordinator;

    /**
     * Creates the ShotCalculator that coordinates everything.
     * 
     * The calculator handles:
     *   1. Phase delay compensation (predict future robot position)
     *   2. Robot-to-shooter transformation (where is the shooter on the robot?)
     *   3. Delegating to the strategy for actual calculation
     */
    public static ShotCalculator createShotCalculator() {
        return new ShotCalculator.Builder()
            .withStrategy(createShotStrategy())
            .withRobotToShooterTransform(LauncherConstants.kRobotToShooter)
            .withPhaseDelay(0.02)  // 20ms processing delay compensation
            .build();
    }

    /** 
     * Creates the InterpolatedShotStrategy with characterization data.
     * 
     * The strategy uses lookup tables (InterpolatingDoubleTreeMap) that map:
     *   - Distance (meters) → Flywheel RPM
     *   - Distance (meters) → Time of Flight (seconds)
     */
    public static InterpolatedShotStrategy createShotStrategy() {
        return new InterpolatedShotStrategy.Builder()
            .withFlywheelSpeedMap(LauncherConstants.createFlywheelSpeedMap())
            .withTimeOfFlightMap(LauncherConstants.createTimeOfFlightMap())
            .withFixedLaunchAngle(kFixedLaunchAngle)
            .withMaxIterations(kMaxIterations)                           // Max lookahead iterations
            .withConvergenceThreshold(kConvergenceThresholdMeters)    // Stop when distance converges within kConvergenceThresholdMeters
            .withValidRange(new Range(kMinRangeMeters, kMaxRangeMeters))  // Valid shooting range
            .build();
    }
}
