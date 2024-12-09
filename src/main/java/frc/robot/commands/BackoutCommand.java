package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class BackoutCommand extends Command {
    private IntakeSubsystem m_IntakeSubsystem;
    private IndexerSubsystem m_IndexerSubsystem;
    private ShooterSubsystem m_ShooterSubsystem;

    public BackoutCommand(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter) {
        m_IntakeSubsystem = intake;
        m_IndexerSubsystem = indexer;
        m_ShooterSubsystem = shooter;
        addRequirements(indexer, intake, shooter);
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
        if(m_IndexerSubsystem.getSensor2() && m_IndexerSubsystem.getSensor())
        {
             m_IntakeSubsystem.setIntake(-TunerConstants.kBackoutSpeed);
             m_IndexerSubsystem.setIndexer(TunerConstants.kBackoutSpeed);
             m_ShooterSubsystem.setRPM(-500);
             m_ShooterSubsystem.moveAngler(TunerConstants.kIndexAngle);

        }
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
        return !m_IndexerSubsystem.getSensor2() && m_IndexerSubsystem.getSensor();
    }

}
