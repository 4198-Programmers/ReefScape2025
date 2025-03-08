package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class ZeroGyro extends Command{

    private SwerveSubsystem swerveSubsystem;
    private PoseEstimatorSubsystem poseEstimatorSubsystem;
    
    public ZeroGyro(SwerveSubsystem swerveSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        swerveSubsystem.resetGyro();
        swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(Math.PI)));
        poseEstimatorSubsystem.resetPoseEstimator();
    }
    
}
