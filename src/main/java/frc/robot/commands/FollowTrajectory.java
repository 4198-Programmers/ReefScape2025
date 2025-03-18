package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class FollowTrajectory extends Command {

    private final SwerveSubsystem m_robotDrive;
    private final SwerveControllerCommand swerveControllerCommand;
    

         // Create config for trajectory
         TrajectoryConfig config =
         new TrajectoryConfig(
                 Constants.kMaxSpeedMetersPerSecond,
                 Constants.kMaxAccelerationMetersPerSecondSquared)
             // Add kinematics to ensure max speed is actually obeyed
             .setKinematics(Constants.SWERVE_DRIVE_KINEMATICS);
 
     // An example trajectory to follow. All units in meters.
     Trajectory exampleTrajectory =
         TrajectoryGenerator.generateTrajectory(
             // Start at the origin facing the +X direction
             Pose2d.kZero,
             // Pass through these two interior waypoints, making an 's' curve path
             List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
             // End 3 meters straight ahead of where we started, facing forward
             new Pose2d(3, 0, Rotation2d.kZero),
             config);
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            1, 0, 0, Constants.kThetaControllerConstraints);

   



    public FollowTrajectory(SwerveSubsystem swerveSubsystem) {
        this.m_robotDrive = swerveSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        
        swerveControllerCommand = new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            Constants.SWERVE_DRIVE_KINEMATICS,

            // Position controllers
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            thetaController,
            (states) -> m_robotDrive.setSwerveModuleStates(states),
            m_robotDrive
        );
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveControllerCommand.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_robotDrive.drive(0, 0, 0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
