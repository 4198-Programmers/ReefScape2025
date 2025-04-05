package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class TrajectoryFollowing extends Command {

    private Trajectory trajectory;
    private final SwerveSubsystem swerveSubsystem;
    HolonomicDriveController controller = new HolonomicDriveController(
        new PIDController(0.5, 0, 0), new PIDController(0.5, 0, 0),
        new ProfiledPIDController(0.0, 0, 0,
        new TrapezoidProfile.Constraints(0.3, 0.1)));
// Here, our rotation profile constraints were a max velocity
// of 1 rotation per second and a max acceleration of 180 degrees
// per second squared.

    public TrajectoryFollowing(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.resetGyro();
        swerveSubsystem.resetOdometryPose();
        swerveSubsystem.resetToAbsolutes();
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        trajectory = generateTrajectory();
    }

    @Override
    public void execute() {

        // Sample the trajectory at the end
        Trajectory.State goal = trajectory.sample(trajectory.getTotalTimeSeconds());
        // Get the adjusted speeds. Here, we want the robot to be facing
        // 70 degrees (in the field-relative coordinate system).
        ChassisSpeeds adjustedSpeeds = controller.calculate(
        swerveSubsystem.getPose(), goal, Rotation2d.fromDegrees(70.0));
        swerveSubsystem.setSwerveModuleStates(Constants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(adjustedSpeeds));
    }

    public Trajectory generateTrajectory() {
        // 2018 cross scale auto waypoints.
        var currentPose = swerveSubsystem.getPose();
    var start = new Pose2d(currentPose.getX(), currentPose.getX(),
        currentPose.getRotation());
    var end = new Pose2d(currentPose.getX() - 0.9, currentPose.getY() - 0.297,
        currentPose.getRotation());

    var interiorWaypoints = new ArrayList<Translation2d>();

    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(1), Units.feetToMeters(1));
    config.setReversed(true);

    var trajectory = TrajectoryGenerator.generateTrajectory(
        start,
        interiorWaypoints,
        end,
        config);
    return trajectory;

    }
 }
