package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ManipulatorSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

public class ManipulatorCommand extends Command {
  private final ManipulatorSubsystem manipulatorSubsystem;
  private Double yValue;
  private Joystick joystick;

  // variable to run during auto, null means we don't run off of it
  private double AutoSpeedOverride;

  /**
   * Creates a new ExampleCommand.
   * Set AutoSpeedOverride to 0 if you don't want to use it
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManipulatorCommand(ManipulatorSubsystem subsystem, Joystick joystick, double AutoSpeedOverride) {
    manipulatorSubsystem = subsystem;
    this.joystick = joystick;

    this.AutoSpeedOverride = AutoSpeedOverride;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("Ran Manipulator Command");
    yValue = joystick.getY();
    if(AutoSpeedOverride != 0) {
      yValue = AutoSpeedOverride;
    }

    // System.out.println(yValue);
    if(yValue > 0.1 || yValue < -0.1) {
      manipulatorSubsystem.turnPrimaryJoint(-yValue * Constants.ManipulatorConstants.MANIPULATOR_MOTOR_SPEED);
    } else {
      manipulatorSubsystem.turnPrimaryJoint(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manipulatorSubsystem.turnPrimaryJoint(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
