// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;


public class IntakeSubsystem extends SubsystemBase {
  public static CANSparkFlex leftIntake = new CANSparkFlex(TunerConstants.kLeftIntakeID,CANSparkLowLevel.MotorType.kBrushless);
  public static CANSparkFlex rightIntake = new CANSparkFlex(TunerConstants.kRightIntakeID, CANSparkLowLevel.MotorType.kBrushless);
 
  // public static DigitalInput sensor = TunerConstants.sensor1;
  public IntakeSubsystem() {
    rightIntake.setInverted(true);
    leftIntake.setInverted(false);
    
  }

  public void setIntake(double spd) {
    leftIntake.set(spd);
    rightIntake.set(spd);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
