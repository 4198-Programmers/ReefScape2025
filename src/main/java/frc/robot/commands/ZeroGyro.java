package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroGyro extends Command{

    private final SwerveSubsystem swerveSubsytem;

    public ZeroGyro(SwerveSubsystem swerveSubsytem) {
        this.swerveSubsytem = swerveSubsytem;
        addRequirements(swerveSubsytem);
    }

    @Override
    public void execute() {
        swerveSubsytem.gyro.reset();
    }
    
}
