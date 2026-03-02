package frc.robot.subsystems.launcher.moveandshoot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

/**
 * NetworkTables interface for launcher tuning and telemetry.
 *
 * <p>Publishes shot calculation results for debugging and characterization.
 */
public class LauncherTuning {

    private final String tableName;

    public LauncherTuning(String tableName) {
        this.tableName = tableName + "/Telemetry/";  // e.g. "BBLauncher/Telemetry/'SomeValue'"
    }

    /**
     * Publish shot parameters to telemetry.
     *
     * Extract values from ShotParameters and publish to NetworkTables.
     * Remember to convert Measure types: shot.flywheelSpeed().in(RPM)
     */
    public void publishTelemetry(ShotParameters shot) {
        // Example: flywheelRPMEntry.setDouble(shot.flywheelSpeed().in(RPM));

        Logger.recordOutput(tableName + "FlywheelSpeed(RPM)", shot.flywheelSpeed().in(RPM));
        Logger.recordOutput(tableName + "LauncherYaw(deg)", shot.launcherYaw().in(Degrees));
        Logger.recordOutput(tableName + "EffectiveDistance(m)", shot.effectiveDistance().in(Meters));
        Logger.recordOutput(tableName + "TimeOfFlight(s)", shot.timeOfFlight().in(Seconds));
        Logger.recordOutput(tableName + "Iterations", shot.iterationsUsed());
        Logger.recordOutput(tableName + "Converged", shot.converged());
        Logger.recordOutput(tableName + "ValidShot", shot.validShot());
        Logger.recordOutput(tableName + "EffectiveTarget", shot.effectiveTarget());
    }
}
