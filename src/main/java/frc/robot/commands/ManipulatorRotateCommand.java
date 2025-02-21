package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.RotateManipulatorSubsystem;

public class ManipulatorRotateCommand extends Command {
    private final RotateManipulatorSubsystem manipulatorSubsystem;

    public ManipulatorRotateCommand(RotateManipulatorSubsystem subsystem) {
    manipulatorSubsystem = subsystem;
    
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    manipulatorSubsystem.toggleRotateIntake();
  }

  @Override
  public void end(boolean interrupted) {
    
  }
}
