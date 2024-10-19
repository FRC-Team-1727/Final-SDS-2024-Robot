// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  private final IntakeSubsystem m_IntakeSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;
  private final IndexerSubsystem m_IndexerSubsystem;

  public IntakeCommand(IntakeSubsystem intake, ShooterSubsystem shooter, IndexerSubsystem indexer) {
    m_IntakeSubsystem = intake;
    m_ShooterSubsystem = shooter;
    m_IndexerSubsystem = indexer;
    addRequirements(m_IndexerSubsystem, m_IntakeSubsystem, m_ShooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.setIntake(0);
    m_IndexerSubsystem.setIndexer(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_IntakeSubsystem.setIntake(TunerConstants.kIntakeSpd);
      m_IndexerSubsystem.setIndexer(-TunerConstants.kIndexerSpd);
      m_ShooterSubsystem.setRPM(-200);
      m_ShooterSubsystem.moveAngler(TunerConstants.kIndexAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setIntake(0);
    m_IndexerSubsystem.setIndexer(0);
    m_ShooterSubsystem.setRPM(0);
    m_ShooterSubsystem.moveAngler(TunerConstants.kDefaultAngle);
  }

  //
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_IndexerSubsystem.getSensor();
  }
}
