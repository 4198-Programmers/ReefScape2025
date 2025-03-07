package frc.robot.commands;

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
        new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
        new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS =
        new TrapezoidProfile.Constraints(8, 8);

    private static final int TAG_TO_CHASE = 6;
    private static final Transform3d TAG_TO_GOAL = new Transform3d(new Translation3d(1.5, 0.0, 0.0), new Rotation3d(0, 0, Math.PI)); // IMPORTANT BECAUSE IN REFERENCE TO TAG

    private final PhotonCamera photonCamera;
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Pose2d> poseProvider;

    private final ProfiledPIDController xController = new ProfiledPIDController(0.5, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(0.5, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(0.5, 0, 0, THETA_CONSTRAINTS);

    private PhotonTrackedTarget target;

    public ChaseTagCommand(PhotonCamera photonCamera, SwerveSubsystem swerveSubsystem, Supplier<Pose2d> poseProvider) {
        this.photonCamera = photonCamera;
        this.swerveSubsystem = swerveSubsystem;
        this.poseProvider = poseProvider;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
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
                .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
                .filter(t -> !t.equals(target) && t.getPoseAmbiguity() <= 0.2)
                .findFirst();
            if (targetOption.isPresent()) {
                target = targetOption.get();

                var cameraPose = robotPose.transformBy(Constants.CAMERA_TO_ROBOT); //Maybe robot to camera?

                var camToTarget = target.getBestCameraToTarget();
                var targetPose = cameraPose.transformBy(camToTarget);

                var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                xController.setGoal(goalPose.getX());
                yController.setGoal(goalPose.getY());
                thetaController.setGoal(goalPose.getRotation().getRadians());
            }
        }
        if (target == null) {
            swerveSubsystem.drive(0, 0, 0, false);
        } else {
            var xSpeed = xController.calculate(robotPose.getX());
            if (xController.atGoal()) {
                xSpeed = 0;
            }
            var ySpeed = yController.calculate(robotPose.getY());
            if (yController.atGoal()) {
                ySpeed = 0;
            }
            var thetaSpeed = thetaController.calculate(robotPose2d.getRotation().getRadians());
            if (thetaController.atGoal()) {
                thetaSpeed = 0;
            }

            swerveSubsystem.drive(xSpeed, ySpeed, thetaSpeed, false);
        }
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
