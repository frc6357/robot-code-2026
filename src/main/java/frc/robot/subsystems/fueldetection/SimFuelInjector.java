package frc.robot.subsystems.fueldetection;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.SKSwerve;

/**
 * Simulation-only subsystem that injects fake fuel positions into a
 * {@link FuelDetection}'s {@link FuelMap} so the whole fuel pipeline
 * (tracking → clustering → scoring → aiming) can be exercised without a
 * real Limelight.
 *
 * <p>When the robot drives within {@link #PICKUP_RADIUS_M} of a sim ball,
 * that ball is removed from both the injection list and the fuel map — 
 * simulating collection.
 *
 * <h2>Usage</h2>
 * <ol>
 *   <li>Instantiate only when {@code Robot.isSimulation()} is true.</li>
 *   <li>Toggle the <b>"Inject Sim Fuel"</b> switch on the Shuffleboard
 *       "Sim Fuel" tab (or via NetworkTables) to start / stop injection.</li>
 * </ol>
 *
 * <p>Edit {@link #INITIAL_FUEL_POSITIONS} to change the layout.
 */
public class SimFuelInjector extends SubsystemBase {

    /** How close the robot must be to "collect" a sim ball (metres). */
    private static final double PICKUP_RADIUS_M = 0.5;

    // ---------------------------------------------------------------
    //  Simulated ball positions  (field-space, metres)
    // ---------------------------------------------------------------

    private static final Translation2d[] INITIAL_FUEL_POSITIONS = {
        // Cluster near field centre
        new Translation2d(8.25, 4.10),
        new Translation2d(8.50, 4.00),
        new Translation2d(8.35, 3.80),

        // Cluster near blue community
        new Translation2d(3.00, 2.50),
        new Translation2d(3.20, 2.60),
        new Translation2d(3.10, 2.30),

        // Lone ball near red side
        new Translation2d(13.50, 5.50),

        // Scattered balls along the trench
        new Translation2d(5.00, 1.20),
        new Translation2d(6.50, 1.10),
        new Translation2d(8.00, 1.30),
        new Translation2d(9.50, 1.15),
    };

    // ---------------------------------------------------------------
    //  State
    // ---------------------------------------------------------------

    private final GenericEntry enabledEntry;
    private final FuelMap fuelMap;
    private final SKSwerve swerve;

    /** Mutable copy — balls are removed as the robot collects them. */
    private final List<Translation2d> remainingFuel = new ArrayList<>();

    // ---------------------------------------------------------------
    //  Constructor
    // ---------------------------------------------------------------

    public SimFuelInjector(FuelDetection fuelDetection, SKSwerve swerve) {
        this.fuelMap = fuelDetection.getFuelMap();
        this.swerve  = swerve;

        // Seed the list
        for (Translation2d pos : INITIAL_FUEL_POSITIONS) {
            remainingFuel.add(pos);
        }

        ShuffleboardTab tab = Shuffleboard.getTab("Sim Fuel");
        enabledEntry = tab.add("Inject Sim Fuel", false)
                          .withWidget(BuiltInWidgets.kToggleSwitch)
                          .withSize(2, 1)
                          .withPosition(0, 0)
                          .getEntry();
    }

    // ---------------------------------------------------------------
    //  Periodic
    // ---------------------------------------------------------------

    @Override
    public void periodic() {
        if (!enabledEntry.getBoolean(false)) {
            return;
        }

        // --- Remove balls the robot has driven over ---
        Translation2d robotPos = swerve.getRobotPose().getTranslation();
        remainingFuel.removeIf(pos -> pos.getDistance(robotPos) <= PICKUP_RADIUS_M);
        fuelMap.removeNear(robotPos, PICKUP_RADIUS_M);

        // --- Inject surviving balls into the fuel map ---
        fuelMap.update(remainingFuel.toArray(new Translation2d[0]));
    }
}
