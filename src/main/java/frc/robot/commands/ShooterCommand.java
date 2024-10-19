package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
    
  /** Creates a new IntakeCommand. */
  private final IntakeSubsystem m_IntakeSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;
  private final IndexerSubsystem m_IndexerSubsystem;

  private double ang;

  public ShooterCommand(IntakeSubsystem intake, ShooterSubsystem shooter, IndexerSubsystem indexer, double ang) {
    m_IntakeSubsystem = intake;
    m_ShooterSubsystem = shooter;
    m_IndexerSubsystem = indexer;
    this.ang = ang;
    addRequirements(m_IndexerSubsystem, m_IntakeSubsystem, m_ShooterSubsystem);
  }

  @Override 
  public void initialize() {
        m_ShooterSubsystem.moveAngler(ang);
  }

  @Override
  public void execute() {
      m_ShooterSubsystem.setRPM(3000);

      if (Math.abs(m_ShooterSubsystem.getRPM() - 3000) < 400) {
        m_IntakeSubsystem.setIntake(TunerConstants.kIntakeSpd);
        m_IndexerSubsystem.setIndexer(-TunerConstants.kIndexerSpd);
      }
  }

  @Override
  public void end(boolean interrupted)
  {
    m_IntakeSubsystem.setIntake(0);
    m_IndexerSubsystem.setIndexer(0);
    m_ShooterSubsystem.setRPM(0);
    m_ShooterSubsystem.moveAngler(TunerConstants.kDefaultAngle);
  }

}
