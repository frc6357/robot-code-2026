package frc.robot.subsystems.pickupOB;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Konstants.pickupOBConstants.*;
import static frc.robot.Ports.pickupOBPorts.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkMax;

@SuppressWarnings("unused")

public class SKpickupOB extends SubsystemBase {
  //Declarations

  // TalonFX positionerMotor;
  //TalonFX eaterMotor; 

  //If Vortex:
  SparkMax positionerMotor;
  SparkMax eaterMotor; 

  //Encoder
  CANcoder encoder;

  //Limit Switches
  //SparkLimitSwitch switch1;
  //SparkLimitSwitch switch2;


  double motorCurrentPosition;
  double motorTargetPosition;

  // Constructor 
  public SKpickupOB() {
    //Initializations
    //positionerMotor = new TalonFX(kPositionerMotor.ID, kPositionerMotor.bus);
    //eaterMotor = new TalonFX(kEaterMotor.ID, kEaterMotor.bus);
    positionerMotor = new SparkMax(kPositionerMotor.ID, MotorType.kBrushless);
    eaterMotor = new SparkMax(kEaterMotor.ID, MotorType.kBrushless);

    // Limit switches with REV
    //forwardLimitSwitch = motorL.getForwardLimitSwitch();
    //reverseLimitSwitch = motorL.getReverseLimitSwitch();

    motorCurrentPosition = kPositionerMotorMinPosition;
    motorTargetPosition = kPositionMotorMaxPosition;

  }

  //Function ideas, getspeed, get position, stop, setspeed,
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // SmartDashboard.putNumber("Velocity (RPMs)", getMotorSpeed());
  }

}
