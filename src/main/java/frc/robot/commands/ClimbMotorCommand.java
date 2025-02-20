package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbMotorSubsystem;

public class ClimbMotorCommand extends Command {
    private ClimbMotorSubsystem climbMotorSubsystem;
    private double speed;

    public ClimbMotorCommand(ClimbMotorSubsystem climbMotorSubsystem, double speed){
        this.climbMotorSubsystem = climbMotorSubsystem;
        this.speed = speed;
        
        addRequirements(climbMotorSubsystem);
    }

    @Override
    public void execute(){
        climbMotorSubsystem.setClimbSpeed(speed);
    }

    @Override
    public void end(boolean interrupted){
        climbMotorSubsystem.stop();
    }

    
}
