package frc.robot.subsystems.pickupOB;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Konstants.pickupOBConstants.*;
import static frc.robot.Ports.pickupOBPorts.*;

import com.ctre.phoenix6.hardware.TalonFX;

@SuppressWarnings("unused")

public class SKpickupOB extends SubsystemBase {
  //Declarations
  //Falcon Motors
  TalonFX positionerMotor;
  TalonFX eaterMotor;
  //If Vortex;
  

  double motorCurrentPosition = kPositionerMotorMinPosition;
  double motorTargetPosition = kPositionMotorMaxPosition;

  // Constructor 
  public SKpickupOB() {
    //Initializations
    positionerMotor = new TalonFX(kPositionerMotor.ID, kPositionerMotor.bus);
    eaterMotor = new TalonFX(kEaterMotor.ID, kEaterMotor.bus);


  }

  //Function ideas, getspeed, get position, stop, setspeed,
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // SmartDashboard.putNumber("Velocity (RPMs)", getMotorSpeed());
  }

}
