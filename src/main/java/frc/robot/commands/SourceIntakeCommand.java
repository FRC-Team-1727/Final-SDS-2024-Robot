package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SourceIntakeCommand extends Command{
    private IndexerSubsystem m_IndexerSubsystem;
    private ShooterSubsystem m_ShooterSubsystem;

    public SourceIntakeCommand(IndexerSubsystem indexer, ShooterSubsystem shooter) {
        m_IndexerSubsystem = indexer;
        m_ShooterSubsystem = shooter;
        addRequirements(m_IndexerSubsystem, m_ShooterSubsystem);
    }

    public void initialize() {
        m_IndexerSubsystem.setIndexer(0);
        m_ShooterSubsystem.setAngler(TunerConstants.kSubwooferAngle);
        m_ShooterSubsystem.setRPM(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(!m_IndexerSubsystem.getSensor2() && !m_IndexerSubsystem.getSensor())
        {
            m_IndexerSubsystem.setIndexer(TunerConstants.kInverseIndexerSpd);
            m_ShooterSubsystem.setRPM(-1000); //tune these numbers tomorrow
        }else if(m_IndexerSubsystem.getSensor2() && m_IndexerSubsystem.getSensor())
        {
            m_IndexerSubsystem.setIndexer(TunerConstants.kInverseIndexerSpd/2);
            m_ShooterSubsystem.setRPM(-500);//tune this one as well
        }
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_IndexerSubsystem.setIndexer(0);
        m_ShooterSubsystem.setAngler(TunerConstants.kDefaultAngle);
        m_ShooterSubsystem.setRPM(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_IndexerSubsystem.getSensor() && !m_IndexerSubsystem.getSensor2();
    }
}
