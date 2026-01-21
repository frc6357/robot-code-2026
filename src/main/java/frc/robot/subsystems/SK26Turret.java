// 120 min, -140 max

//Subsystem Essentials
package frc.robot.subsystems;

import static frc.robot.Konstants.TurretConstants.kTurretP;
import static frc.robot.Konstants.TurretConstants.kTurretI;
import static frc.robot.Konstants.TurretConstants.kTurretD;
import static frc.robot.Konstants.TurretConstants.kTurretMinPosition;
import static frc.robot.Konstants.TurretConstants.kTurretMaxPosition;
import static frc.robot.Konstants.TurretConstants.kTurretAngleTolerance;
import static frc.robot.Ports.LauncherPorts.kTurretMotor;

//import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VelocityDutyCycle;
// import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Velocity;
//import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
//import edu.wpi.first.units.Units;
//import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
//SmartDashboard Import
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.preferences.Pref;
import frc.robot.preferences.SKPreferences;

public class SK26Turret extends SubsystemBase 
{
    // Launcher motor delcaration.
    TalonFX turretMotor;
    TalonFXConfiguration motorConfig;
    MotorOutputConfigs outputConfigs;

    // Launcher PID controller declarations.
    PhoenixPIDController turretPID;
    Slot0Configs turretPID0;

    // Angle tracking declarations.
    double motorTargetPosition;
    double motorCurrentPosition;
    double target;
    boolean atTarget;
    
    // Simulation stuff
    PositionVoltage why;
    NeutralOut neutral = new NeutralOut();

    // PID Preferences.
    Pref<Double> turretkPPref = SKPreferences.attach("turretP", 1.0)
        .onChange((newValue) -> turretPID.setP(newValue));
    Pref<Double> turretkIPref = SKPreferences.attach("turretI", 0.0)
        .onChange((newValue) -> turretPID.setI(newValue));
    Pref<Double> turretkDPref = SKPreferences.attach("turretD", 0.0)
        .onChange((newValue) -> turretPID.setD(newValue));

    public SK26Turret() 
    {
        // Motor initialization
        turretMotor = new TalonFX(kTurretMotor.ID);
        motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // PID Controller initialization
        turretPID = new PhoenixPIDController(kTurretP, kTurretI, kTurretD);
        turretPID.reset();
        turretPID.setTolerance(kTurretAngleTolerance);

        turretPID0 = new Slot0Configs()
            .withKP(turretkPPref.get())
            .withKI(turretkIPref.get())
            .withKD(turretkDPref.get());

        motorConfig.withSlot0(turretPID0);
        motorCurrentPosition = kTurretMinPosition;
        motorTargetPosition = kTurretMaxPosition;
    }

    public void setBrake() 
    {
        turretMotor.setControl(neutral);
    }

    //If we Eyeball
    public void runMotor(double turretSpeed) 
    {
        if((getMotorPosition() > (kTurretMaxPosition - kTurretAngleTolerance)) && Math.signum(turretSpeed) > 0) 
        {
            stop();
            return;
        }
        else
        { 
            // Sets a velocity to target via pid and supplies an average duty cycle in volts
            turretMotor.setControl(new DutyCycleOut(turretSpeed));//.withFeedForward(3.0)); // FF in volts
        }
    }

    // Simmulation
    private static final double kGearRatio = 3.0;

    private final DCMotorSim m_motorSimModel = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
        DCMotor.getKrakenX60Foc(1), 
        0.001, 
        kGearRatio),
    DCMotor.getKrakenX60Foc(1)
    );

    @Override
    public void simulationPeriodic() 
    {
        var talonFXSim = turretMotor.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage.in(Units.Volts));
        m_motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
        talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));
    }

    public double getMotorSpeed() 
    {
        return turretMotor.getVelocity().getValueAsDouble(); // Rotations / sec
    }

    public double getMotorPosition() 
    {
        return turretMotor.getPosition().getValueAsDouble(); // Rotations
    }

    public double getTargetPosition() 
    {
        return turretMotor.getClosedLoopReference().getValueAsDouble();
    }

    public void stop() 
    {
        turretMotor.stopMotor();
    }

    public boolean isAtTargetPosition() 
    {
        return Math.abs(getTargetPosition() - getMotorPosition()) <= kTurretAngleTolerance;
    }

    @Override
    public void periodic() 
    {
        motorCurrentPosition = getMotorPosition(); 

        SmartDashboard.putNumber("Turret Velocity", getMotorSpeed());
        SmartDashboard.putNumber("Current Turret Position", getMotorPosition());
        SmartDashboard.putNumber("Target Turret Position", getTargetPosition());
    }
}