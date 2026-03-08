package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
    @AutoLog
    class DriveIOInputs {
        // Gyro
        public double pitchDegrees;
        public double pitchVelocityDegreesPerSecond;
        public double rollDegrees;
        public double rollVelocityDegreesPerSecond;

        // Per-module arrays (index 0-3)
        public double[] driveVolts = new double[4];
        public double[] steerVolts = new double[4];
        public double[] driveStatorCurrentAmps = new double[4];
        public double[] steerStatorCurrentAmps = new double[4];
        public double[] driveSupplyCurrentAmps = new double[4];
        public double[] steerSupplyCurrentAmps = new double[4];
        public double[] driveTempCelsius = new double[4];
        public double[] steerTempCelsius = new double[4];
    }

    void updateInputs(DriveIOInputs inputs);
}
