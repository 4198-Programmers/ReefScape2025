// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ClimbMotorCommand;
import frc.robot.subsystems.ClimbMotorSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManipulatorCommand;
import frc.robot.commands.ManipulatorPositionOne;
import frc.robot.commands.ManipulatorRotateCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.RotateManipulatorSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorSteadyCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();
  private final RotateManipulatorSubsystem rotateManipulatorSubsystem = new RotateManipulatorSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  // Subsystems
  private final ClimbMotorSubsystem climbMotorSubsystem = new ClimbMotorSubsystem();

  // Joysticks
  private final Joystick leftJoystick = new Joystick(Constants.JOYSTICK_LEFT_ID);
  private final Joystick middleJoystick = new Joystick(Constants.JOYSTICK_MIDDLE_ID);
  private final Joystick rightJoystick = new Joystick(Constants.JOYSTICK_RIGHT_ID);

  // Buttons
  private final JoystickButton climbButton = new JoystickButton(rightJoystick, Constants.ClimbConstants.CLIMB_FORWARD_BUTTON);
  private final JoystickButton climbButtonReverse = new JoystickButton(rightJoystick, Constants.ClimbConstants.CLIMB_REVERSE_BUTTON);

  private final JoystickButton elevatorUpButton = new JoystickButton(rightJoystick, Constants.ElevatorConstants.ELEVATOR_UP_BUTTON);
  private final JoystickButton elevatorDownButton = new JoystickButton(rightJoystick, Constants.ElevatorConstants.ELEVATOR_DOWN_BUTTON);



    private JoystickButton manipulatorRotateButton = new JoystickButton(rightJoystick, Constants.RIGHT_JOYSTICK_BUTTON_TWELVE);
    private JoystickButton intakeButton = new JoystickButton(rightJoystick, Constants.INTAKE_BUTTON);
    private JoystickButton outtakeButton = new JoystickButton(rightJoystick, Constants.OUTTAKE_BUTTON);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    manipulatorSubsystem.setDefaultCommand(new ManipulatorCommand(manipulatorSubsystem, rightJoystick));
    // Configure the trigger bindings
    configureBindings();
  }

  // Configure Button Bindings
  private void configureBindings() {
    climbButton.whileTrue(new ClimbMotorCommand(climbMotorSubsystem, Constants.ClimbConstants.CLIMB_SPEED));
    climbButtonReverse.whileTrue(new ClimbMotorCommand(climbMotorSubsystem, -Constants.ClimbConstants.CLIMB_SPEED));
    
    elevatorUpButton.whileTrue(new ElevatorCommand(m_elevatorSubsystem,-Constants.ElevatorConstants.ELEVATOR_SPEED));
    elevatorDownButton.whileTrue(new ElevatorCommand(m_elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SPEED));
    elevatorUpButton.whileFalse(new ElevatorSteadyCommand(m_elevatorSubsystem));
    elevatorDownButton.whileFalse(new ElevatorSteadyCommand(m_elevatorSubsystem));
  }

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    // manipulatorRotateButton.whileTrue(new ManipulatorPositionOne(manipulatorSubsystem));
    manipulatorRotateButton.toggleOnTrue(new ManipulatorRotateCommand(rotateManipulatorSubsystem));
    intakeButton.whileTrue(new IntakeCommand(intakeSubsystem, -Constants.ManipulatorConstants.INTAKE_MOTOR_SPEED));
    outtakeButton.whileTrue(new IntakeCommand(intakeSubsystem, Constants.ManipulatorConstants.INTAKE_MOTOR_SPEED));
  

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
