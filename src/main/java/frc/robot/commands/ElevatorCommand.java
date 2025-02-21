package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
        // System.out.println(elevatorSubsystem.checkSwitch());

        // Checks switch and if elevator is moving up and switch is pressed, stop moving, 
        if (speed < 0 && elevatorSubsystem.checkSwitch()) {
            elevatorSubsystem.move(0);
        } else {
            elevatorSubsystem.move(speed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.move(0);
        elevatorSubsystem.setSteadyEncoderPosition(); // Logs the encoder value for the steady command
    }
}
    

