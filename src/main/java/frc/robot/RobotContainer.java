// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ClimbMotorCommand;
import frc.robot.commands.ClimbMotorReverseCommand;
import frc.robot.subsystems.ClimbMotorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorSteadyCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  // Subsystems
  private final ClimbMotorSubsystem climbMotorSubsystem = new ClimbMotorSubsystem();

  // Joysticks
  private final Joystick leftJoystick = new Joystick(Constants.JOYSTICK_LEFT_ID);
  private final Joystick middleJoystick = new Joystick(Constants.JOYSTICK_MIDDLE_ID);
  private final Joystick rightJoystick = new Joystick(Constants.JOYSTICK_RIGHT_ID);

  // Buttons
  private final JoystickButton climbButton = new JoystickButton(rightJoystick, 6);
  private final JoystickButton climbButtonReverse = new JoystickButton(rightJoystick, 4);

  //create a button object 3 and 5
  private JoystickButton elevatorUpButton = new JoystickButton(rightJoystick, Constants.RIGHT_JOYSTICK_BUTTON_FIVE);
  private JoystickButton elevatorDownButton = new JoystickButton(rightJoystick, Constants.RIGHT_JOYSTICK_BUTTON_THREE);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  // Configure Button Bindings
  private void configureBindings() {
    climbButton.whileTrue(new ClimbMotorCommand(climbMotorSubsystem));
    climbButtonReverse.whileTrue(new ClimbMotorReverseCommand(climbMotorSubsystem));
    
    elevatorUpButton.whileTrue(new ElevatorCommand(m_elevatorSubsystem,-Constants.ElevatorConstants.ELEVATOR_SPEED));
    elevatorDownButton.whileTrue(new ElevatorCommand(m_elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SPEED));
    elevatorUpButton.whileFalse(new ElevatorSteadyCommand(m_elevatorSubsystem));
    elevatorDownButton.whileFalse(new ElevatorSteadyCommand(m_elevatorSubsystem));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
