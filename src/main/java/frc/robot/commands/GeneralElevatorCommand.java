package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class GeneralElevatorCommand extends Command {

    private ElevatorSubsystem elevatorSubsystem;
    private boolean up;

    public GeneralElevatorCommand(ElevatorSubsystem elevatorSubsystem, boolean up){
        this.elevatorSubsystem = elevatorSubsystem;
        this.up = up;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.changeMotorPosition(up);
    }
    
}
