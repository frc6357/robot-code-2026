package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.units.Units;

public class DriveIOCTRE implements DriveIO {
    private final GeneratedDrivetrain drivetrain;

    public DriveIOCTRE(GeneratedDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        Pigeon2 pigeon = drivetrain.getPigeon2();

        inputs.pitchDegrees = pigeon.getPitch().getValue().in(Units.Degrees);
        inputs.pitchVelocityDegreesPerSecond =
                pigeon.getAngularVelocityYDevice().getValue().in(Units.DegreesPerSecond);
        inputs.rollDegrees = pigeon.getRoll().getValue().in(Units.Degrees);
        inputs.rollVelocityDegreesPerSecond =
                pigeon.getAngularVelocityXDevice().getValue().in(Units.DegreesPerSecond);

        var modules = drivetrain.getModules();
        for (int i = 0; i < 4; i++) {
            SwerveModule<?, ?, ?> module = modules[i];

            inputs.driveVolts[i] =
                    module.getDriveMotor().getMotorVoltage().getValue().in(Units.Volts);
            inputs.steerVolts[i] =
                    module.getSteerMotor().getMotorVoltage().getValue().in(Units.Volts);
            inputs.driveStatorCurrentAmps[i] =
                    module.getDriveMotor().getStatorCurrent().getValue().in(Units.Amps);
            inputs.steerStatorCurrentAmps[i] =
                    module.getSteerMotor().getStatorCurrent().getValue().in(Units.Amps);
            inputs.driveSupplyCurrentAmps[i] =
                    module.getDriveMotor().getSupplyCurrent().getValue().in(Units.Amps);
            inputs.steerSupplyCurrentAmps[i] =
                    module.getSteerMotor().getSupplyCurrent().getValue().in(Units.Amps);
            inputs.driveTempCelsius[i] =
                    module.getDriveMotor().getDeviceTemp().getValue().in(Units.Celsius);
            inputs.steerTempCelsius[i] =
                    module.getSteerMotor().getDeviceTemp().getValue().in(Units.Celsius);
        }
    }
}
