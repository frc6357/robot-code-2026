package frc.robot.subsystems.pickupOB;

import static frc.robot.Konstants.pickupOBConstants.kEaterMotorSpeed;
import static frc.robot.Konstants.pickupOBConstants.kPositionMotorMaxPosition;
import static frc.robot.Konstants.pickupOBConstants.kPositionerMotorMinPosition;
import static frc.robot.Konstants.pickupOBConstants.kPositionerMotorSpeed;
import static frc.robot.Ports.pickupOBPorts.kEaterMotor;
import static frc.robot.Ports.pickupOBPorts.kPositionerMotor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SK26PickupOB extends SubsystemBase {
 //Declarations

  // TalonFX positionerMotor;
  //TalonFX eaterMotor; 

  //If Vortex:
  SparkFlex positionerMotor;
  SparkFlex eaterMotor; 

  //Encoder
  RelativeEncoder positionerEncoder;
  RelativeEncoder eaterEncoder;

  //Limit Switches
  SparkLimitSwitch forwardLimitSwitch;
  SparkLimitSwitch reverseLimitSwitch;


  double motorCurrentPosition;
  double motorTargetPosition;

  // Constructor 
  public SK26PickupOB() {
    //Initializations

    //positionerMotor = new TalonFX(kPositionerMotor.ID, kPositionerMotor.bus);
    //eaterMotor = new TalonFX(kEaterMotor.ID, kEaterMotor.bus);
    positionerMotor = new SparkFlex(kPositionerMotor.ID, MotorType.kBrushless);
    eaterMotor = new SparkFlex(kEaterMotor.ID, MotorType.kBrushless);

    positionerEncoder = positionerMotor.getEncoder();
    eaterEncoder = eaterMotor.getEncoder();

    // Limit switches with REV
    forwardLimitSwitch = positionerMotor.getForwardLimitSwitch();
    reverseLimitSwitch = positionerMotor.getReverseLimitSwitch();

     //forwardLimitSwitch.enableLimitSwitch(false);
     //reverseLimitSwitch.enableLimitSwitch(false);

    motorCurrentPosition = kPositionerMotorMinPosition;
    motorTargetPosition = kPositionMotorMaxPosition;

  }

  public void runPositionerMotor() {
    positionerMotor.set(kPositionerMotorSpeed);
  }
  public void stopPositionerMotor() {
    positionerMotor.set(0);
  }
  public void runPositionerMotorReverse() {
    positionerMotor.set(-kPositionerMotorSpeed);
  }

  public void runEaterMotor() {
    eaterMotor.set(kEaterMotorSpeed);
  }
  public void stopEaterMotor() {
    eaterMotor.set(0);
  }
  public void runEaterMotorReverse() {
    eaterMotor.set(-kEaterMotorSpeed);
  }

  public void limitStopPositioner() {
    if (forwardLimitSwitch.isPressed() || reverseLimitSwitch.isPressed()) {
      stopPositionerMotor();
    }
  }

  

  //Function ideas, getspeed, get position, stop, setspeed,
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (forwardLimitSwitch.isPressed() || reverseLimitSwitch.isPressed()) {
      stopPositionerMotor();
    }

    SmartDashboard.putNumber("Positioner Velocity (RPMs)", positionerMotor.get());
    SmartDashboard.putNumber("Eater Velocity (RPMs)", eaterMotor.get());

    SmartDashboard.putBoolean("Forward Limit Switch", forwardLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Reverse Limit Switch", reverseLimitSwitch.isPressed());



  }

}

// 