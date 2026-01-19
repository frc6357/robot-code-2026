package frc.robot.subsystems;

import static frc.robot.Konstants.LauncherConstants.kLauncherP;
import static frc.robot.Konstants.LauncherConstants.kLauncherI;
import static frc.robot.Konstants.LauncherConstants.kLauncherD;
import static frc.robot.Konstants.LauncherConstants.kLauncherV;
import static frc.robot.Konstants.LauncherConstants.kWheelRadius;
import static frc.robot.Konstants.LauncherConstants.kShooterTolerance;
import static frc.robot.Konstants.LauncherConstants.kStopLauncher;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Ports.LauncherPorts.kFixedLauncherMotor;

public class SK26Launcher extends SubsystemBase {

    //initialize launcher motor
    TalonFX launchermotor;

    //some other variables
    private final VelocityDutyCycle launcherVelocityControl = new VelocityDutyCycle(0);
    private double targetLaunchVelocity; //meters per second
    private boolean shooting = false;
    
    public SK26Launcher() {

        launchermotor = new TalonFX(kFixedLauncherMotor.ID);
        configMotor();
    }

    public void configMotor() {

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kLauncherP;
        config.Slot0.kI = kLauncherI;
        config.Slot0.kD = kLauncherD;
        config.Slot0.kV = kLauncherV;

        launchermotor.getConfigurator().apply(config);
        launchermotor.setNeutralMode(NeutralModeValue.Brake);
    }

    /*public void startLauncher(double targetLaunchVelocity) {

        this.targetLaunchVelocity = targetLaunchVelocity;
        //Math is probably incredibly wrong but I tried
        double motorRPS = targetLaunchVelocity/(2*Math.PI*kWheelRadius);
        launcherVelocityControl.Velocity = motorRPS; 
        fixedlaunchermotor.setControl(launcherVelocityControl); //makes launcher launch
        shooting = true;
    }*/

    //for debugging the motor ONLY!!
    public void startLauncher(double targetLaunchVelocity) {
        this.targetLaunchVelocity = targetLaunchVelocity;
        launchermotor.set(targetLaunchVelocity);
        shooting = true;
    }
    
    public void unJamLauncher(double speed) {
        startLauncher(speed);
    }

    public boolean isLauncherAtSpeed() {

        double motorRPS = launchermotor.getVelocity().getValueAsDouble();
        double targetRPS = targetLaunchVelocity/(2*Math.PI*kWheelRadius);
        return Math.abs(motorRPS - targetRPS) < kShooterTolerance;
    }

    public void stopLauncher() {

        launchermotor.set(kStopLauncher);
        targetLaunchVelocity = kStopLauncher;
        shooting = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Launcher: ", shooting);
        //Debugging the launcher
        SmartDashboard.putBoolean("Is at launcher speed", isLauncherAtSpeed());
        SmartDashboard.putNumber("Velocity", launchermotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("targetRPS", targetLaunchVelocity/(2*Math.PI*kWheelRadius));

    }
}
