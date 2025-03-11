package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double speed;

    public OuttakeCommand(IntakeSubsystem subsystem, double speed) {
        intakeSubsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        // System.out.println("ran intake");
        intakeSubsystem.runOutake(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.runOutake(0);
    }
    
}