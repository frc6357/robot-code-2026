package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import frc.robot.Ports.LauncherPorts.kFixedLauncherMotor;

public class SKFixedLauncher extends SubsystemBase {

    //initialize launcher motor
    TalonFX fixedlaunchermotor;

    //intiialize PID values
    private static final double kLauncherP = 0;
    private static final double kLauncherI = 0;
    private static final double kLauncherD = 0;
    private static final double kLauncherV = 0;

    //some other variables
    private final VelocityDutyCycle launcherVelocityControl = new VelocityDutyCycle(0);
    private final double kWheelRadius = 0; //meters
    private double launchVelocity;
    private final double shooterTolerance = 0.5;
    private boolean shooting = false;
    
    SKFixedLauncher() {

        fixedlaunchermotor = new TalonFX(0);
        configMotor();
    }

    public void configMotor() {

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kLauncherP;
        config.Slot0.kI = kLauncherI;
        config.Slot0.kD = kLauncherD;
        config.Slot0.kV = kLauncherV;

        fixedlaunchermotor.getConfigurator().apply(config);
        fixedlaunchermotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void startLauncher(double launchVelocity) {

        this.launchVelocity = launchVelocity;
        //Math is probably incredibly wrong but I tried
        double motorRPS = launchVelocity/(2*Math.PI*kWheelRadius);
        launcherVelocityControl.Velocity = motorRPS; 
        fixedlaunchermotor.setControl(launcherVelocityControl); //makes launcher launch
        shooting = true;
    }

    public boolean isLauncherAtSpeed() {

        double motorRPS = fixedlaunchermotor.getVelocity().getValueAsDouble();
        double targetRPS = launchVelocity/(2*Math.PI*kWheelRadius);
        return Math.abs(motorRPS - targetRPS) < shooterTolerance;
    }

    public void stopLauncher() {

        fixedlaunchermotor.set(0);
        shooting = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Fixed Launcher", shooting);
    }
}
