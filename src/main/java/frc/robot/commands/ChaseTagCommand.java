package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class ChaseTagCommand extends Command {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
        new TrapezoidProfile.Constraints(0.5, 1);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
        new TrapezoidProfile.Constraints(0.5, 1);
    private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS =
        new TrapezoidProfile.Constraints(2, 8);

    private List<Integer> TAG_TO_CHASE = new ArrayList<>(List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22));
    private final Transform3d tagToGoal; // IMPORTANT BECAUSE IN REFERENCE TO TAG

    private final PhotonCamera photonCamera;
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Pose2d> poseProvider;

    private final ProfiledPIDController xController = new ProfiledPIDController(0.5, 0.0, 0.1, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(0.5, 0.0, 0.1, Y_CONSTRAINTS);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(0.5, 0, 0.2, THETA_CONSTRAINTS);

    private PhotonTrackedTarget target;

    public ChaseTagCommand(PhotonCamera photonCamera, SwerveSubsystem swerveSubsystem, Supplier<Pose2d> poseProvider, Transform3d tagToGoal) {
        this.tagToGoal = tagToGoal;
        this.photonCamera = photonCamera;
        this.swerveSubsystem = swerveSubsystem;
        this.poseProvider = poseProvider;

        xController.setTolerance(0.005);
        yController.setTolerance(0.005);
        thetaController.setTolerance(Units.degreesToRadians(3));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveSubsystem);
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling getController() here
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        target = null;
        var robotPose = poseProvider.get();
        thetaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var robotPose2d = poseProvider.get();
        var robotPose = new Pose3d(robotPose2d.getX(), robotPose2d.getY(), 0, new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));

        var photonRes = photonCamera.getLatestResult();
        if (photonRes.hasTargets()) {
            var targetOption = photonRes.getTargets().stream()
                .filter(t -> TAG_TO_CHASE.contains(t.getFiducialId()))
                .filter(t -> !t.equals(target) && t.getPoseAmbiguity() <= 0.2)
                .findFirst();
            if (targetOption.isPresent()) {
                target = targetOption.get();

                var cameraPose = robotPose.transformBy(Constants.CAMERA_TO_ROBOT); //Maybe robot to camera?

                var camToTarget = target.getBestCameraToTarget();
                var targetPose = cameraPose.transformBy(camToTarget);

                var goalPose = targetPose.transformBy(tagToGoal).toPose2d();

                xController.setGoal(goalPose.getX());
                yController.setGoal(goalPose.getY());
                thetaController.setGoal(goalPose.getRotation().getRadians());
                // System.out.println("Goal: " + goalPose);
            }
        }
        if (target == null) {
            swerveSubsystem.drive(0, 0, 0, false);
        } else {
            var xSpeed = xController.calculate(robotPose.getX());
            if (xController.atGoal() || Math.abs(xSpeed) < 0.001) {
                xSpeed = 0;
            }
            var ySpeed = yController.calculate(robotPose.getY());
            if (yController.atGoal() || Math.abs(ySpeed) < 0.001) {
                ySpeed = 0;
            }
            var thetaSpeed = thetaController.calculate(robotPose2d.getRotation().getRadians());
            if (thetaController.atGoal()) {
                thetaSpeed = 0;
            }
            // System.out.println("X: " + xSpeed + " Y: " + ySpeed + " Theta: " + thetaSpeed);
            // System.out.println("X is at goal: " + xController.atGoal() + " Y is at goal: " + yController.atGoal() + " Theta is at goal: " + thetaController.atGoal());
            swerveSubsystem.drive(ySpeed, xSpeed, thetaSpeed * 0.04, false);
            return;
        }
        swerveSubsystem.drive(0, 0, 0, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(0, 0, 0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
