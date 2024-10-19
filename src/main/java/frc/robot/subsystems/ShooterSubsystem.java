// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {
   public static CANSparkFlex leftShooter = new CANSparkFlex(TunerConstants.kLeftShooterID,
         CANSparkLowLevel.MotorType.kBrushless);
   public static CANSparkFlex rightShooter = new CANSparkFlex(TunerConstants.kRightShooterID,
         CANSparkLowLevel.MotorType.kBrushless);
   public static CANSparkFlex angler = new CANSparkFlex(TunerConstants.kPivotID, CANSparkLowLevel.MotorType.kBrushless);

   public InterpolatingDoubleTreeMap angleMap;

   public ShooterSubsystem() {
      angler.restoreFactoryDefaults();
      leftShooter.setInverted(true);
      rightShooter.setInverted(false);
      SparkPIDController controller = leftShooter.getPIDController();
      controller.setP(TunerConstants.kShooterP);
      controller.setI(TunerConstants.kShooterI);
      controller.setD(TunerConstants.kShooterD);
      controller = rightShooter.getPIDController();
      controller.setP(TunerConstants.kShooterP);
      controller.setI(TunerConstants.kShooterI);
      controller.setD(TunerConstants.kShooterD);
      controller = angler.getPIDController();
      controller.setP(TunerConstants.kAnglerP);
      controller.setI(TunerConstants.kAnglerI);
      controller.setD(TunerConstants.kAnglerD);
      controller.setFeedbackDevice(angler.getEncoder());
      //controller.setOutputRange(0,0); //We can use this to guess the ranges on the angler(maybe) btw but idk how to print the angle;
      //controller.setIZone(0); //Integral stuff that we could implement just a placeholder for now
      //controller.setIMaxAccum(0, 0); //same with previous stuff only add if have time
      angler.burnFlash();
      leftShooter.burnFlash();
      rightShooter.burnFlash();

      setAnglerAngle(0);
      moveAngler(TunerConstants.kDefaultAngle);

      angleMap = new InterpolatingDoubleTreeMap();
      // dist, angle
      angleMap.put(1.1, 6.47);
      angleMap.put(1.9, 9.4);
      angleMap.put(2.629, 9.92);
   }

   public void setAnglerAngle(double angle)
   {
      angler.getEncoder().setPosition(angle);
   }
   public void moveAngler(double angle){
      angler.getPIDController().setReference(angle, ControlType.kPosition);
   }

   public void setRPM(int rpm) {
      leftShooter.getPIDController().setReference(-rpm, ControlType.kVelocity);
      rightShooter.getPIDController().setReference(-rpm, ControlType.kVelocity);
   }

   public void setAngler(double spd) {
      angler.set(spd);
   }

   public double getRPM() {
      return -leftShooter.getEncoder().getVelocity();
   }

   public double getEncoder() {
      return angler.getEncoder().getPosition();
   }

   @Override
   public void periodic() {

      System.out.println("Angle : " + angler.getEncoder().getPosition());
   }
}
