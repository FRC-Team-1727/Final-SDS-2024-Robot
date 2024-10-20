package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldLayout;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RotateToTargetCommand extends Command{

    CommandSwerveDrivetrain drivetrain;
    PIDController controller;
    private double MaxAngularRate = 1.5 * Math.PI;

    public RotateToTargetCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        controller = new PIDController(0.1, 0, 0);
        controller.setTolerance(3); // how many degrees of tolerance before you wanna continue
        
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double currentAngle = FieldLayout.angleDiffToSpeaker(drivetrain.getState().Pose);
        double output = controller.calculate(currentAngle, FieldLayout.angleDiffToSpeaker(drivetrain.getState().Pose));
        if (Math.abs(output) > MaxAngularRate) {
            output = Math.signum(output) * MaxAngularRate;
        }
        SwerveRequest.FieldCentric rotate = new SwerveRequest.FieldCentric().withRotationalRate(output);
        drivetrain.applyRequest(() -> rotate);
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
}