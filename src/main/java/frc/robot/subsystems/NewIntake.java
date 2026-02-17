// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import static frc.robot.Ports.IntakePorts.*;
import static frc.robot.Konstants.NewIntakeConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NewIntake extends SubsystemBase {
  SparkMax intakeMotor;
  SparkMax intakeMotor2;
  SparkMax intakeMotor3;
  
  public NewIntake() 
  {
    intakeMotor = new SparkMax(kIntakeMotor.ID, MotorType.kBrushless);
    intakeMotor2 = new SparkMax(kIntakeMotor.ID, MotorType.kBrushless);
    intakeMotor3 = new SparkMax(kIntakeMotor.ID, MotorType.kBrushless);
  }

  public void runNewIntake()
  {
    intakeMotor.set(kSetIntakeSpeed);
    intakeMotor2.set(-kSetIntakeSpeed);
    intakeMotor3.set(kSetIntakeSpeed);
  }
    public void stopNewIntake()
  {
    intakeMotor.set(0);
    intakeMotor2.set(0);
    intakeMotor3.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
