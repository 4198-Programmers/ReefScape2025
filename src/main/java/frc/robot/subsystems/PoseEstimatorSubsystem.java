package frc.robot.subsystems;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class PoseEstimatorSubsystem extends SubsystemBase {

    private final SwerveSubsystem swerveSubsystem;
    private final PhotonCamera photonCamera;

    private static final List<Pose3d> targetPoses = Collections.unmodifiableList(List.of(
        new Pose3d(3.0, 0.0, 0.0, new Rotation3d(0, 0, Math.toRadians(180))),
        new Pose3d(3.0, 1.165, 0.287, new Rotation3d(0, 0, Math.toRadians(180)))
    ));

    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    private static final Vector<N3> localMeasurementStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Field2d field2d = new Field2d();
    private double previousPipelineTimestamp = 0;

    PoseEstimatorSubsystem(SwerveSubsystem swerveSubsystem, PhotonCamera photonCamera) {
        this.swerveSubsystem = swerveSubsystem;
        this.photonCamera = photonCamera;
        poseEstimator = new SwerveDrivePoseEstimator(Constants.SWERVE_DRIVE_KINEMATICS, swerveSubsystem.gyro.getRotation2d(), swerveSubsystem.getSwerveModulePositions(), swerveSubsystem.getPose());
        // poseEstimator = new SwerveDrivePoseEstimator<>(N7.class, N7.class, N5.class, swerveSubsystem.getKinematics(), stateStdDevs,
                // localMeasurementStdDevs, visionMeasurementStdDevs, swerveSubsystem.getPose(), swerveSubsystem.getModuleStates());
    }
        // photonCamera.getOutput().addListener((output) -> {
        //     if (output.hasTargets()) {
        //         var target = output.getBestTarget();
        //         var timestamp = output.getTimestamp();
        //         var dt = timestamp - previousPipelineTimestamp;
        //         previousPipelineTimestamp = timestamp;

        //         var targetPose = targetPoses.get(target.getId());
        //         poseEstimator.addVisionMeasurement(
        //             new N5(targetPose.getTranslation().getX(), targetPose.getTranslation().getY(), targetPose.getTranslation().getZ(),
        //                 targetPose.getRotation().getRadians(), targetPose.getRotation().getRadians()),
        //             target.getCameraToTarget(), dt);
        //     }
        // }, true); 

    
    public void periodic() {
        var pipelineResult = photonCamera.getLatestResult();
        var resultTimestamp = pipelineResult.getTimestampSeconds();
        if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
            var target = pipelineResult.getBestTarget();
            var fiducialId = target.getFiducialId();
            if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && fiducialId < targetPoses.size()) {
                var targetPose = targetPoses.get(fiducialId);
                Transform3d camToTarget = target.getBestCameraToTarget();
                Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

                var visionMeasurement = camPose.transformBy(Constants.CAMERA_TO_ROBOT);
                poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
            }
        }
        poseEstimator.update(swerveSubsystem.gyro.getRotation2d(), swerveSubsystem.getSwerveModulePositions());

        field2d.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public String getFormattedPose() {
        var pose = getCurrentPose();
        return String.format("(%.2f, %.2f) %.2f degrees", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }
    
}
