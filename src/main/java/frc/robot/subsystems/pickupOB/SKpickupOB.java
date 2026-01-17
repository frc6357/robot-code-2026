package frc.robot.subsystems.pickupOB;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Konstants.pickupOBConstants.*;
import static frc.robot.Ports.pickupOBPorts.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

@SuppressWarnings("unused")

public class SKpickupOB extends SubsystemBase {
  //Declarations

  // TalonFX positionerMotor;
  //TalonFX eaterMotor; 

  //If Vortex:
  SparkFlex positionerMotor;
  SparkFlex eaterMotor; 

  double motorCurrentPosition;
  double motorTargetPosition;

  // Constructor 
  public SKpickupOB() {
    //Initializations
    //positionerMotor = new TalonFX(kPositionerMotor.ID, kPositionerMotor.bus);
    //eaterMotor = new TalonFX(kEaterMotor.ID, kEaterMotor.bus);
    positionerMotor = new SparkFlex(kPositionerMotor.ID, MotorType.kBrushless);
    eaterMotor = new SparkFlex(kEaterMotor.ID, MotorType.kBrushless);

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
