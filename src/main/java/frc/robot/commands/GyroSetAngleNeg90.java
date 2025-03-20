package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class GyroSetAngleNeg90 extends Command{

    private SwerveSubsystem swerveSubsystem;

    public GyroSetAngleNeg90(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        swerveSubsystem.setGyroAngle(-45);
    }
    
}
