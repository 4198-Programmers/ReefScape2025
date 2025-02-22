package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RotateManipulatorSubsystem;

public class ManipulatorRotateCommand extends Command {
    private final RotateManipulatorSubsystem manipulatorSubsystem;

    public ManipulatorRotateCommand(RotateManipulatorSubsystem subsystem) {
    manipulatorSubsystem = subsystem;
    
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    // Toggles it to the other rotation value when the command is initialized
    manipulatorSubsystem.toggleRotate();
    // System.out.println("initialized");
  }

  @Override
  public void execute() {
    // keeps running the rotation to get to the target position
    manipulatorSubsystem.toggleRotateIntake();

  }

  @Override
  public void end(boolean interrupted) {
    manipulatorSubsystem.stopRotate();
  }
}
