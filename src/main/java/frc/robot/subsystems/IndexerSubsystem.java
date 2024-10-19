// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class IndexerSubsystem extends SubsystemBase {
  public static CANSparkFlex index = new CANSparkFlex(TunerConstants.kIndexerID, CANSparkLowLevel.MotorType.kBrushless);
  private DigitalInput sensor = new DigitalInput(TunerConstants.kSensorID);
  private DigitalInput sensor2 = new DigitalInput(TunerConstants.kSensorID2);
  // public static DigitalInput sensor = TunerConstants.sensor1;

  public void setIndexer(double spd) {
     index.set(spd);
  }
  public boolean getSensor()
  {
    return !sensor.get();
  }
  public boolean getSensor2()
  {
    return !sensor2.get();
  }

  public IndexerSubsystem() {
  }

  @Override
  public void periodic() {
    System.out.println("sensor 1 : " + getSensor());
    System.out.println("sensor 2 : " + getSensor2());
  }
}
