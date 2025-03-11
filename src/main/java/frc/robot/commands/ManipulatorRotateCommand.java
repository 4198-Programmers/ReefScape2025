package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RotateManipulatorSubsystem;

public class ManipulatorRotateCommand extends Command {

    private RotateManipulatorSubsystem rotateManipulatorSubsystem;
    private double speed;

    public ManipulatorRotateCommand(RotateManipulatorSubsystem rotateManipulatorSubsystem, double speed) {
        this.rotateManipulatorSubsystem = rotateManipulatorSubsystem;
        addRequirements(rotateManipulatorSubsystem);
    }

    public void execute() {
        rotateManipulatorSubsystem.RotateManipulator(speed);
    }
    
}
