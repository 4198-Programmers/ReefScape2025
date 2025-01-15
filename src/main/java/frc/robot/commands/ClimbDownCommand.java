package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbMotorSubsystem;

public class ClimbDownCommand extends Command{
    
    private ClimbMotorSubsystem motorSubsystem;

    public void ClimbDownCommand(ClimbMotorSubsystem motorSubsystem) {
        this.motorSubsystem = motorSubsystem;
        addRequirements(motorSubsystem);
    }

    @Override
    public void execute() {
        motorSubsystem.climb(-1);
    }
}
