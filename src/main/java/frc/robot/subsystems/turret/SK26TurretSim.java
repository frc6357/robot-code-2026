package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Konstants.TurretConstants.kEncoderGearRatio;
import static frc.robot.Konstants.TurretConstants.kTurretMotorGearRatio;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Simulation implementation of the turret subsystem.
 * Overrides motor feedback to use DCMotorSim physics simulation.
 */
public class SK26TurretSim extends SK26Turret {
    
    // Moment of inertia for the turret (kg⋅m²)
    // Higher values = more resistance to acceleration, slower response, less oscillation
    // Typical turret MOI: 0.05 to 0.5 kg⋅m² depending on mass and radius
    private static final double kTurretMOI = 0.0375; // Adjust to match real turret behavior
    
    private final DCMotorSim turretMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44Foc(1), kTurretMOI, kTurretMotorGearRatio), 
        DCMotor.getKrakenX44Foc(1)
    );

    // Reference to motor and encoder sim states
    private final TalonFXSimState turretSimState;
    private final CANcoderSimState encoderSimState;

    public SK26TurretSim() {
        super();
        
        // Get the sim state from the parent's motor (turretMotor is protected)
        turretSimState = turretMotor.getSimState();
        // Use Clockwise_Positive because kTurretMotorInverted=true means 
        // negative voltage produces positive angle change
        turretSimState.Orientation = ChassisReference.Clockwise_Positive;
        turretSimState.setMotorType(MotorType.KrakenX44);
        turretSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Get the sim state from the parent's encoder
        encoderSimState = turretEncoder.getSimState();
        encoderSimState.Orientation = ChassisReference.CounterClockwise_Positive;
        
        // Initialize encoder sim state to 0 (matching DCMotorSim initial state)
        encoderSimState.setRawPosition(0);
        encoderSimState.setVelocity(0);
        
        // CRITICAL: Reset the PID controller and sync target to current position
        // The parent constructor ran before encoder sim was initialized, which may have
        // caused incorrect initial readings and PID integral accumulation
        resetPIDController();
    }

    @Override
    public double getMotorDutyCycle() {
        // In simulation, use the simulated motor voltage divided by supply voltage to get duty cycle
        double supplyVoltage = RobotController.getBatteryVoltage();
        if (supplyVoltage == 0) {
            return 0; // Avoid division by zero
        }
        return turretSimState.getMotorVoltageMeasure().in(Volts) / supplyVoltage;
    }

    @Override
    public double getMotorVoltage() {
        // In simulation, use the simulated motor voltage
        return turretSimState.getMotorVoltageMeasure().in(Volts);
    }

    @Override
    public void periodic() {
        // Run the parent's periodic first (PID loop calculates and applies voltage)
        super.periodic();
        
        // Then update simulation physics with the voltage that was just applied
        // This ensures the next cycle reads the updated position
        updateSimulation();
    }

    /**
     * Updates the motor simulation with physics calculations.
     */
    private void updateSimulation() {
        // Set supply voltage of the motor
        turretSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Get the motor voltage of the KrakenX44
        var motorVoltage = turretSimState.getMotorVoltageMeasure();

        // Use motor's voltage to calculate new position and velocity using WPILib's DCMotorSim
        turretMotorSim.setInputVoltage(motorVoltage.in(Volts));
        turretMotorSim.update(0.02); // Update simulation with 20ms timestep

        // Apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        turretSimState.setRawRotorPosition(turretMotorSim.getAngularPosition().times(kTurretMotorGearRatio));
        turretSimState.setRotorVelocity(turretMotorSim.getAngularVelocity().times(kTurretMotorGearRatio));

        // Update CANcoder sim state - this is what the PID actually reads!
        // DCMotorSim gives mechanism position (turret angle), CANcoder needs encoder rotations
        // Turret has a 2:1 encoder gear ratio (2 encoder rotations = 1 turret rotation)
        double turretAngleDeg = turretMotorSim.getAngularPosition().in(Degrees);
        double encoderRotations = turretAngleDeg / (360.0 / kEncoderGearRatio);
        encoderSimState.setRawPosition(encoderRotations);
        
        // Also set encoder velocity
        double turretVelocityDegPerSec = turretMotorSim.getAngularVelocity().in(Degrees.per(edu.wpi.first.units.Units.Second));
        double encoderVelocityRotPerSec = turretVelocityDegPerSec / (360.0 / kEncoderGearRatio);
        encoderSimState.setVelocity(encoderVelocityRotPerSec);
    }
}
