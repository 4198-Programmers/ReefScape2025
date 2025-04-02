package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

public class AutoManipulatorCommand extends Command{

    private ManipulatorSubsystem manipulatorSubsystem;

    public AutoManipulatorCommand(ManipulatorSubsystem manipulatorSubsystem) {
        this.manipulatorSubsystem = manipulatorSubsystem;
        addRequirements(manipulatorSubsystem);
    }

    @Override
    public void execute() {
        manipulatorSubsystem.turnPrimaryJointToPosition(41.8805);
    }
    
}
