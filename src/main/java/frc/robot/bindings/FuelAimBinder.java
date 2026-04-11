package frc.robot.bindings;

import static frc.robot.Ports.DriverPorts.kStartbutton;

import java.util.Optional;

import frc.lib.bindings.CommandBinder;
import frc.robot.commands.automatedDriving.AimAtFuelCommand;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.fueldetection.FuelDetection;

/**
 * Binds the driver Start button to {@link AimAtFuelCommand} so that while
 * the button is held the robot's heading locks toward the best fuel cluster.
 * The driver keeps full robot-centric translation — just push forward to collect.
 */
public class FuelAimBinder implements CommandBinder {

    private final Optional<SKSwerve>      drive;
    private final Optional<FuelDetection> fuelDetection;

    public FuelAimBinder(Optional<SKSwerve> drive, Optional<FuelDetection> fuelDetection) {
        this.drive         = drive;
        this.fuelDetection = fuelDetection;
    }

    @Override
    public void bindButtons() {
        if (!drive.isPresent() || !fuelDetection.isPresent()) {
            return;
        }

        kStartbutton.button.whileTrue(
            new AimAtFuelCommand(drive.get(), fuelDetection.get())
                .withName("AimAtFuel")
        );
    }
}
