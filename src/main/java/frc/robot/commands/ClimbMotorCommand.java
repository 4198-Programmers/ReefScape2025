package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbMotorSubsystem;

public class ClimbMotorCommand extends Command {
    private ClimbMotorSubsystem climbMotorSubsystem;

    public ClimbMotorCommand(ClimbMotorSubsystem climbMotorSubsystem){
        this.climbMotorSubsystem = climbMotorSubsystem;
        
        addRequirements(climbMotorSubsystem);
    }

    @Override
    public void execute(){
        climbMotorSubsystem.setClimbSpeed(Constants.CLIMB_SPEED);
    }

    @Override
    public void end(boolean interrupted){
        climbMotorSubsystem.stop();
    }

    
}
