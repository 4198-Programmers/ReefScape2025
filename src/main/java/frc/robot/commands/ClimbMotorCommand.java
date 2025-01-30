package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
        this.climbMotorSubsystem.setClimbSpeed(this.speed);
    }

    @Override
    public void end(boolean interrupted){
        this.climbMotorSubsystem.stop();
    }

    
}
