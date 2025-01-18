package frc.robot.commands.ledFolder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedSubsystem;

public class LedCommand extends Command {

    private LedSubsystem ledSubsystem;
    
    // private Command circleSillyTime(boolean StupidIdiot) {
    //     return runOnce(
    //             () -> {
    //                 circleMode = StupidIdiot;
    //             });
    // }
    
public LedCommand(LedSubsystem ledSubsystem){
    this.ledSubsystem = ledSubsystem;
    addRequirements(ledSubsystem);
    }

    @Override
    public void initialize(){
        
    }
    @Override
    public void execute(){
        ledSubsystem.test();
    }
    
    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
