package frc.robot.commands;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class OuttakeCommand extends Command {
    private IndexerSubsystem m_IndexerSubsystem;
    private IntakeSubsystem m_IntakeSubsystem;
    private ShooterSubsystem m_ShooterSubsystem;

    public OuttakeCommand(IndexerSubsystem indexer, IntakeSubsystem intake, ShooterSubsystem shooter) {
        m_IndexerSubsystem = indexer;
        m_IntakeSubsystem = intake;
        m_ShooterSubsystem = shooter;
        addRequirements(m_IndexerSubsystem, m_IntakeSubsystem, m_ShooterSubsystem);
    }

    public void initialize() {
        m_IntakeSubsystem.setIntake(0);
        m_IndexerSubsystem.setIndexer(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_IntakeSubsystem.setIntake(-TunerConstants.kOuttakeSpd);
        m_IndexerSubsystem.setIndexer(TunerConstants.kOuttakeSpd);
        m_ShooterSubsystem.moveAngler(TunerConstants.kIndexAngle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.setIntake(0);
        m_IndexerSubsystem.setIndexer(0);
        m_ShooterSubsystem.moveAngler(TunerConstants.kDefaultAngle);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
