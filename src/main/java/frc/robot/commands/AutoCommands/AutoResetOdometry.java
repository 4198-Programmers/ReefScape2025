package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class AutoResetOdometry extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final double x, y, z;

    public AutoResetOdometry(SwerveSubsystem swerveSubsystem, double x, double y, double z) {
        this.swerveSubsystem = swerveSubsystem;
        this.x = x;
        this.y = y;
        this.z = z;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.resetPose(new Pose2d(x, y, Rotation2d.fromDegrees(z)));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
