// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.BackoutCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.StuckCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  

  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final RobotStateEstimator estimator = new RobotStateEstimator(drivetrain);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

   // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    joystick.rightTrigger().whileTrue(new ShooterCommand(m_IntakeSubsystem, m_ShooterSubsystem, m_IndexerSubsystem, 9.41));
    // joystick.rightTrigger().whileTrue(new ShooterCommand(m_IntakeSubsystem, m_ShooterSubsystem, m_IndexerSubsystem, 
    //                                                               m_ShooterSubsystem.angleMap.get(FieldLayout.distanceFromAllianceWall(drivetrain.getState().Pose.getX(), false))));
    joystick.rightBumper().whileTrue(new IntakeCommand(m_IntakeSubsystem, m_ShooterSubsystem, m_IndexerSubsystem).andThen(new BackoutCommand(m_IntakeSubsystem, m_IndexerSubsystem, m_ShooterSubsystem)));
    // joystick.povLeft().whileTrue(new OuttakeCommand(m_IndexerSubsystem, m_IntakeSubsystem, m_ShooterSubsystem));
    joystick.povUp().whileTrue(new OuttakeCommand(m_IndexerSubsystem, m_IntakeSubsystem, m_ShooterSubsystem));
    joystick.y().whileTrue(new StuckCommand(m_IndexerSubsystem, m_IntakeSubsystem, m_ShooterSubsystem));

  }

  public ShooterSubsystem getShooterSubsystem()
  {
    return m_ShooterSubsystem;
  }


  public void setInitPose(Pose2d pose) {
    drivetrain.seedFieldRelative(pose);
    // drivetrain.getState().Pose = pose;
    drivetrain.getPigeon2().setYaw(pose.getRotation().getDegrees());
  }

  public RobotContainer() {
    configureBindings();
  }
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
