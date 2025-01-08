package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.BallIntakeSubsystem;

public class BallOutputCommand extends Command {
    // Initialize BallIntakeSubsystem
    BallIntakeSubsystem ballIntakeSubsystem;

    public BallOutputCommand(BallIntakeSubsystem ballGrabberSubsystem) {
        this.ballIntakeSubsystem = ballGrabberSubsystem;

        // Optimizes command
        addRequirements(ballGrabberSubsystem);
    }

    // Execute the command at a negative speed
    @Override
    public void execute() {
        ballIntakeSubsystem.setGrabberMotorSpeed(-Constants.BALL_INTAKE_SPEED);
    }

    // Set motor speed to 0 when command ends
    @Override
    public void end(boolean interrupted) {
        ballIntakeSubsystem.stop();
    }
}
