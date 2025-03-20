package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonSubsystem extends SubsystemBase {
    public PhotonCamera camera = new PhotonCamera("frontcam");

    public PhotonSubsystem() { }

    public void takeSnapshot() {
        var results = camera.getLatestResult();

        if (results.hasTargets()) {
            for (var target : results.getTargets()) {
                if (target.fiducialId == 6) {
                    System.out.println(target.bestCameraToTarget);
                }
                // System.out.println("Target found: " + target);
            }
        }

        // System.out.println(camera.getAllUnreadResults());
    }

    public PhotonTrackedTarget getTarget() {
        var results = camera.getLatestResult();
        if (results.hasTargets()) {
            for (var target : results.getTargets()) {
                if (target.fiducialId == 6) {
                    return target;
                }
                // System.out.println("Target found: " + target);
            }
        }
        return null;
    }

    public Command takeShot() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                takeSnapshot();
            }
        );
    }

    public void periodic() {
        // This method will be called once per scheduler run
    }
}
