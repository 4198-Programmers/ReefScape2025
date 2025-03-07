package frc.robot.commands;

import org.photonvision.proto.Photon;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
public class PhotonVisionCommand extends Command {

    private final PhotonSubsystem photonSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    public PhotonVisionCommand(PhotonSubsystem photonSubsystem, SwerveSubsystem swerveSubsystem) {
        this.photonSubsystem = photonSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(photonSubsystem);
    }

    @Override
    public void execute() {
        // photonSubsystem.takeSnapshot();
        var target = photonSubsystem.getTarget();
        if (target != null) {
            // System.out.println("Target found: " + target);
            var yaw = target.getYaw();
            System.out.println("Yaw: " + yaw);
            if (Math.abs(yaw) < 2) {
                swerveSubsystem.drive(0, 0, 0, false);
                return;
            }
            System.out.println("Speed: " + yaw * 0.005);
            swerveSubsystem.drive(0, 0, -yaw * 0.005, false);

        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(0, 0, 0, false);
    }
    
}
