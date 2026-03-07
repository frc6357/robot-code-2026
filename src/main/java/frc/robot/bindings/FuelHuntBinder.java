package frc.robot.bindings;

import java.util.Optional;

import static frc.robot.Ports.DriverPorts.kAButton;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.commandGroups.FuelHuntCommand;
import frc.robot.subsystems.drive.SKSwerve;
import frc.robot.subsystems.fueldetection.FuelDetection;

/**
 * Binds the FuelHunt command group to a driver button.
 *
 * <p><b>A button (hold)</b> — runs the corridor-based fuel collection system.
 * The command drives out through the nearest trench, sweeps through the
 * neutral zone on one continuous spline collecting fuel, and returns through
 * the opposite trench. Releasing the button cancels at any point.
 */
public class FuelHuntBinder implements CommandBinder {

    private final Optional<SKSwerve> m_swerve;
    private final Optional<FuelDetection> m_fuelDetection;

    private final Trigger fuelHuntButton = kAButton.button;

    public FuelHuntBinder(Optional<SKSwerve> m_swerve, Optional<FuelDetection> m_fuelDetection) {
        this.m_swerve = m_swerve;
        this.m_fuelDetection = m_fuelDetection;
    }

    @Override
    public void bindButtons() {
        if (!m_swerve.isPresent() || !m_fuelDetection.isPresent()) {
            return;
        }

        // Hold A to run FuelHunt; releasing cancels immediately
        fuelHuntButton.whileTrue(
            FuelHuntCommand.create().withName("FuelCollection_DriverButton")
        );
    }
}
