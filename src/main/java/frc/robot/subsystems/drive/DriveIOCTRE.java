package frc.robot.subsystems.drive;

import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class DriveIOCTRE implements DriveIO {
    private static final long REFRESH_INTERVAL_MS = 20;

    private final StatusSignal<Angle> pitchSignal;
    private final StatusSignal<AngularVelocity> pitchVelocitySignal;
    private final StatusSignal<Angle> rollSignal;
    private final StatusSignal<AngularVelocity> rollVelocitySignal;

    private final StatusSignal<Voltage>[] driveVoltageSignals;
    private final StatusSignal<Voltage>[] steerVoltageSignals;
    private final StatusSignal<Current>[] driveStatorCurrentSignals;
    private final StatusSignal<Current>[] steerStatorCurrentSignals;
    private final StatusSignal<Current>[] driveSupplyCurrentSignals;
    private final StatusSignal<Current>[] steerSupplyCurrentSignals;
    private final StatusSignal<Temperature>[] driveTempSignals;
    private final StatusSignal<Temperature>[] steerTempSignals;

    private final BaseStatusSignal[] allSignals;
    private final AtomicReference<DriveIOInputs> latestSnapshot = new AtomicReference<>(new DriveIOInputs());

    @SuppressWarnings("unchecked")
    public DriveIOCTRE(GeneratedDrivetrain drivetrain) {
        var pigeon = drivetrain.getPigeon2();
        pitchSignal = pigeon.getPitch(false);
        pitchVelocitySignal = pigeon.getAngularVelocityYDevice(false);
        rollSignal = pigeon.getRoll(false);
        rollVelocitySignal = pigeon.getAngularVelocityXDevice(false);

        var modules = drivetrain.getModules();
        driveVoltageSignals = new StatusSignal[4];
        steerVoltageSignals = new StatusSignal[4];
        driveStatorCurrentSignals = new StatusSignal[4];
        steerStatorCurrentSignals = new StatusSignal[4];
        driveSupplyCurrentSignals = new StatusSignal[4];
        steerSupplyCurrentSignals = new StatusSignal[4];
        driveTempSignals = new StatusSignal[4];
        steerTempSignals = new StatusSignal[4];

        for (int i = 0; i < 4; i++) {
            driveVoltageSignals[i] = modules[i].getDriveMotor().getMotorVoltage(false);
            steerVoltageSignals[i] = modules[i].getSteerMotor().getMotorVoltage(false);
            driveStatorCurrentSignals[i] = modules[i].getDriveMotor().getStatorCurrent(false);
            steerStatorCurrentSignals[i] = modules[i].getSteerMotor().getStatorCurrent(false);
            driveSupplyCurrentSignals[i] = modules[i].getDriveMotor().getSupplyCurrent(false);
            steerSupplyCurrentSignals[i] = modules[i].getSteerMotor().getSupplyCurrent(false);
            driveTempSignals[i] = modules[i].getDriveMotor().getDeviceTemp(false);
            steerTempSignals[i] = modules[i].getSteerMotor().getDeviceTemp(false);
        }

        allSignals = new BaseStatusSignal[] {
            pitchSignal, pitchVelocitySignal, rollSignal, rollVelocitySignal,
            driveVoltageSignals[0], driveVoltageSignals[1], driveVoltageSignals[2], driveVoltageSignals[3],
            steerVoltageSignals[0], steerVoltageSignals[1], steerVoltageSignals[2], steerVoltageSignals[3],
            driveStatorCurrentSignals[0], driveStatorCurrentSignals[1], driveStatorCurrentSignals[2], driveStatorCurrentSignals[3],
            steerStatorCurrentSignals[0], steerStatorCurrentSignals[1], steerStatorCurrentSignals[2], steerStatorCurrentSignals[3],
            driveSupplyCurrentSignals[0], driveSupplyCurrentSignals[1], driveSupplyCurrentSignals[2], driveSupplyCurrentSignals[3],
            steerSupplyCurrentSignals[0], steerSupplyCurrentSignals[1], steerSupplyCurrentSignals[2], steerSupplyCurrentSignals[3],
            driveTempSignals[0], driveTempSignals[1], driveTempSignals[2], driveTempSignals[3],
            steerTempSignals[0], steerTempSignals[1], steerTempSignals[2], steerTempSignals[3],
        };

        var thread = new Thread(this::refreshLoop, "DriveIOCTRE-CAN");
        thread.setDaemon(true);
        thread.start();
    }

    private void refreshLoop() {
        while (!Thread.interrupted()) {
            try {
                BaseStatusSignal.refreshAll(allSignals);

                var snapshot = new DriveIOInputs();
                snapshot.pitchDegrees = pitchSignal.getValue().in(Units.Degrees);
                snapshot.pitchVelocityDegreesPerSecond = pitchVelocitySignal.getValue().in(Units.DegreesPerSecond);
                snapshot.rollDegrees = rollSignal.getValue().in(Units.Degrees);
                snapshot.rollVelocityDegreesPerSecond = rollVelocitySignal.getValue().in(Units.DegreesPerSecond);

                for (int i = 0; i < 4; i++) {
                    snapshot.driveVolts[i] = driveVoltageSignals[i].getValue().in(Units.Volts);
                    snapshot.steerVolts[i] = steerVoltageSignals[i].getValue().in(Units.Volts);
                    snapshot.driveStatorCurrentAmps[i] = driveStatorCurrentSignals[i].getValue().in(Units.Amps);
                    snapshot.steerStatorCurrentAmps[i] = steerStatorCurrentSignals[i].getValue().in(Units.Amps);
                    snapshot.driveSupplyCurrentAmps[i] = driveSupplyCurrentSignals[i].getValue().in(Units.Amps);
                    snapshot.steerSupplyCurrentAmps[i] = steerSupplyCurrentSignals[i].getValue().in(Units.Amps);
                    snapshot.driveTempCelsius[i] = driveTempSignals[i].getValue().in(Units.Celsius);
                    snapshot.steerTempCelsius[i] = steerTempSignals[i].getValue().in(Units.Celsius);
                }

                latestSnapshot.set(snapshot);
                Thread.sleep(REFRESH_INTERVAL_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        var snapshot = latestSnapshot.get();

        inputs.pitchDegrees = snapshot.pitchDegrees;
        inputs.pitchVelocityDegreesPerSecond = snapshot.pitchVelocityDegreesPerSecond;
        inputs.rollDegrees = snapshot.rollDegrees;
        inputs.rollVelocityDegreesPerSecond = snapshot.rollVelocityDegreesPerSecond;

        System.arraycopy(snapshot.driveVolts, 0, inputs.driveVolts, 0, 4);
        System.arraycopy(snapshot.steerVolts, 0, inputs.steerVolts, 0, 4);
        System.arraycopy(snapshot.driveStatorCurrentAmps, 0, inputs.driveStatorCurrentAmps, 0, 4);
        System.arraycopy(snapshot.steerStatorCurrentAmps, 0, inputs.steerStatorCurrentAmps, 0, 4);
        System.arraycopy(snapshot.driveSupplyCurrentAmps, 0, inputs.driveSupplyCurrentAmps, 0, 4);
        System.arraycopy(snapshot.steerSupplyCurrentAmps, 0, inputs.steerSupplyCurrentAmps, 0, 4);
        System.arraycopy(snapshot.driveTempCelsius, 0, inputs.driveTempCelsius, 0, 4);
        System.arraycopy(snapshot.steerTempCelsius, 0, inputs.steerTempCelsius, 0, 4);
    }
}
