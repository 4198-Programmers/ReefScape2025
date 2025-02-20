package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSteadyCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private double encoderValue;

    public ElevatorSteadyCommand(ElevatorSubsystem elevatorSubsystem) {
        // Makes a new instance of ElevatorSubsystem each time command in run
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        encoderValue = elevatorSubsystem.getSteadyEncoderPosition();
        if (encoderValue < elevatorSubsystem.checkEncoder()) {
            elevatorSubsystem.move(-0.025); // Moves very slowly up so there isn't any jittering
        } else {
            elevatorSubsystem.move(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.move(0);
    }
}
