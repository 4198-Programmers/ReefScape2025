// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorSteadyCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  //Create a Joystick object
  private final Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_ID);

  //create a button object 3 and 5
  private JoystickButton elevatorUpButton = new JoystickButton(rightJoystick, Constants.RIGHT_JOYSTICK_BUTTON_FIVE);
  private JoystickButton elevatorDownButton = new JoystickButton(rightJoystick, Constants.RIGHT_JOYSTICK_BUTTON_THREE);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    elevatorUpButton.whileTrue(new ElevatorCommand(m_elevatorSubsystem,-Constants.ElevatorConstants.ELEVATOR_SPEED));
    elevatorDownButton.whileTrue(new ElevatorCommand(m_elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SPEED));
    elevatorUpButton.whileFalse(new ElevatorSteadyCommand(m_elevatorSubsystem));
    elevatorDownButton.whileFalse(new ElevatorSteadyCommand(m_elevatorSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
