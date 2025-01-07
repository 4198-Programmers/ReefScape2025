// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.BallIntakeCommand;
import frc.robot.commands.BallOutputCommand;
import frc.robot.subsystems.BallIntakeSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // Subsystem for getAutoCommand
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Subsystems
  private final BallIntakeSubsystem ballIntakeSubsystem = new BallIntakeSubsystem();

  // Joysticks
  Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_ID);
  Joystick middleJoystick = new Joystick(Constants.MIDDLE_JOYSTICK_ID);
  Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_ID);

  // Joystick Buttons
  JoystickButton ballIntakeButton = new JoystickButton(leftJoystick, Constants.JOYSTICK_BUTTON_1);
  JoystickButton ballOutputButton = new JoystickButton(leftJoystick, Constants.JOYSTICK_BUTTON_2);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }
  
  private void configureBindings() {
    ballIntakeButton.whileTrue(new BallIntakeCommand(ballIntakeSubsystem));
    ballOutputButton.whileTrue(new BallOutputCommand(ballIntakeSubsystem));
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
