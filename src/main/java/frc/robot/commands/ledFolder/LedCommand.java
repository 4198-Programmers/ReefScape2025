package frc.robot.commands.ledFolder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.NewLedSubsytem;

public class LedCommand extends Command {

    private NewLedSubsytem newLedSubsystem;
    
    // private Command circleSillyTime(boolean StupidIdiot) {
    //     return runOnce(
    //             () -> {
    //                 circleMode = StupidIdiot;
    //             });
    // }
    
public LedCommand(NewLedSubsytem newLedSubsystem){
    this.newLedSubsystem = newLedSubsystem;
    addRequirements(newLedSubsystem);
    }

    @Override
    public void initialize(){
        
    }
    @Override
    public void execute(){
        newLedSubsystem.RunLEDS();
    }
    
    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
