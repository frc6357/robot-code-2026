// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import static frc.robot.Ports.IntakePorts.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NewIntake extends SubsystemBase {
  SparkMax intakeMotor;
  
  public NewIntake() {
    intakeMotor = new SparkMax(kIntakeMotor.ID, MotorType.kBrushless);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
