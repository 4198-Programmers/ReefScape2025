package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {

    private ElevatorSubsystem elevatorSubsystem;
    private double speed;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double speed) {
        // Makes a new instance of ElevatorSubsystem each time command in run
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;

        addRequirements(elevatorSubsystem);
    }
        @Override 
        public void execute() {
            elevatorSubsystem.move(speed);
        }
    }
    

