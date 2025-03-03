package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class ResetToAbsolutes extends Command {

    private SwerveSubsystem swerveSubsystem;
    
    public ResetToAbsolutes(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        swerveSubsystem.resetToAbsolutes();
    }

}
