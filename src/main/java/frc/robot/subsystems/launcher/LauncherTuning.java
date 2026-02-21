package frc.robot.subsystems.launcher;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * NetworkTables interface for launcher tuning and telemetry.
 *
 * <p>Publishes shot calculation results for debugging and characterization.
 */
public class LauncherTuning {

    private final NetworkTable table;

    // Telemetry entries
    private final NetworkTableEntry flywheelRPMEntry;
    private final NetworkTableEntry launcherYawEntry;
    private final NetworkTableEntry distanceEntry;
    private final NetworkTableEntry timeOfFlightEntry;
    private final NetworkTableEntry iterationsEntry;
    private final NetworkTableEntry convergedEntry;
    private final NetworkTableEntry validShotEntry;

    public LauncherTuning(String tableName) {
        this.table = NetworkTableInstance.getDefault().getTable(tableName);

        // TODO: STUDENT EXERCISE - Set up telemetry entries
        // Create entries for each value you want to publish.
        // Use table.getEntry("Telemetry/Name") to create entries.

        this.flywheelRPMEntry = table.getEntry("Telemetry/FlywheelRPM");
        this.launcherYawEntry = table.getEntry("Telemetry/LauncherYaw");
        this.distanceEntry = table.getEntry("Telemetry/Distance");
        this.timeOfFlightEntry = table.getEntry("Telemetry/TimeOfFlight");
        this.iterationsEntry = table.getEntry("Telemetry/Iterations");
        this.convergedEntry = table.getEntry("Telemetry/Converged");
        this.validShotEntry = table.getEntry("Telemetry/ValidShot");
    }

    /**
     * Publish shot parameters to telemetry.
     *
     * TODO: STUDENT EXERCISE - Implement telemetry publishing
     * Extract values from ShotParameters and publish to NetworkTables.
     * Remember to convert Measure types: shot.flywheelSpeed().in(RPM)
     */
    public void publishTelemetry(ShotParameters shot) {
        // TODO: Publish each value
        // Example: flywheelRPMEntry.setDouble(shot.flywheelSpeed().in(RPM));

        flywheelRPMEntry.setDouble(shot.flywheelSpeed().in(RPM));
        launcherYawEntry.setDouble(shot.launcherYaw().in(Degrees));
        distanceEntry.setDouble(shot.effectiveDistance().in(Meters));
        timeOfFlightEntry.setDouble(shot.timeOfFlight().in(Seconds));
        iterationsEntry.setDouble(shot.iterationsUsed());
        convergedEntry.setBoolean(shot.converged());
        validShotEntry.setBoolean(shot.validShot());

        // Also publish to SmartDashboard for easy viewing
        SmartDashboard.putBoolean("Launcher/ValidShot", shot.validShot());
        SmartDashboard.putNumber("Launcher/Distance", shot.effectiveDistance().in(Meters));
    }
}
