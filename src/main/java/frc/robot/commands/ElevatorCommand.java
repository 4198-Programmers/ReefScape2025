package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {

    private ElevatorSubsystem elevatorSubsystem;
    private int elevatorPosition;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, int elevatorPosition) {
        // Makes a new instance of ElevatorSubsystem each time command in run
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorPosition = elevatorPosition;
        addRequirements(elevatorSubsystem);
    }
    
    @Override 
    public void execute() {
        switch (elevatorPosition) {
            case 0:
                elevatorSubsystem.moveToPosition(ElevatorConstants.ELEVATOR_POSITION_0);
                break;
            case 1:
                elevatorSubsystem.moveToPosition(ElevatorConstants.ELEVATOR_POSITION_1);
                break;
            case 2:
                elevatorSubsystem.moveToPosition(ElevatorConstants.ELEVATOR_POSITION_2);
                break;
            case 3:
                elevatorSubsystem.moveToPosition(ElevatorConstants.ELEVATOR_POSITION_3);
                break;
            default:
                break;
        }
    }



    @Override
    public void end(boolean interrupted) {
        // elevatorSubsystem.setSteadyEncoderPosition(); // Logs the encoder value for the steady command
    }
}
    

