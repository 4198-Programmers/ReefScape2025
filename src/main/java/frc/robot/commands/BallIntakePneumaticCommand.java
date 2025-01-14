package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallIntakePneumaticSubsystem;

public class BallIntakePneumaticCommand extends Command {
    BallIntakePneumaticSubsystem ballIntakePnuematicSubsystem;

    public BallIntakePneumaticCommand(BallIntakePneumaticSubsystem ballIntakePnuematicSubsystem) {
        this.ballIntakePnuematicSubsystem = ballIntakePnuematicSubsystem;

        addRequirements(ballIntakePnuematicSubsystem);
    }

    @Override
    public void initialize() {
        ballIntakePnuematicSubsystem.initialize();
    }

    @Override
    public void execute() {
        ballIntakePnuematicSubsystem.intakeDown();
    }

    @Override
    public void end(boolean interrupted) {
        ballIntakePnuematicSubsystem.intakeUp();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
