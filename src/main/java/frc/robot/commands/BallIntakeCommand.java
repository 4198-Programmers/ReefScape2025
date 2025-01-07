package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.BallIntakeSubsystem;

public class BallIntakeCommand extends Command {
    BallIntakeSubsystem ballGrabberSubsystem;

    public BallIntakeCommand(BallIntakeSubsystem ballGrabberSubsystem) {
        this.ballGrabberSubsystem = ballGrabberSubsystem;

        addRequirements(ballGrabberSubsystem);
    }

    @Override
    public void execute() {
        ballGrabberSubsystem.setGrabberMotorSpeed(Constants.BALL_GRABBER_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        ballGrabberSubsystem.stop();
    }

    // @Override
    // public boolean isFinished() {
    //     return RobotContainer.RobotContainer()
    // }
}
