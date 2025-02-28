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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorSetPositionsCommand;
import frc.robot.commands.ElevatorSteadyCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ManipulatorConstants;

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

    private final JoystickButton elevatorPositionOne = new JoystickButton(middleJoystick, Constants.ElevatorConstants.ELEVATOR_BUTTON_POSITION_ONE);
    private final JoystickButton elevatorPositionTwo = new JoystickButton(middleJoystick, Constants.ElevatorConstants.ELEVATOR_BUTTON_POSITION_TWO);
    private final JoystickButton elevatorPositionThree = new JoystickButton(middleJoystick, Constants.ElevatorConstants.ELEVATOR_BUTTON_POSITION_THREE);
    private final JoystickButton elevatorPositionFour = new JoystickButton(middleJoystick, Constants.ElevatorConstants.ELEVATOR_BUTTON_POSITION_FOUR);

    private JoystickButton manipulatorRotateButton = new JoystickButton(rightJoystick, ManipulatorConstants.MANIPULATOR_ROTATE_BUTTON);
    private JoystickButton intakeButton = new JoystickButton(rightJoystick, Constants.INTAKE_BUTTON);
    private JoystickButton outtakeButton = new JoystickButton(rightJoystick, Constants.OUTTAKE_BUTTON);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        manipulatorSubsystem.setDefaultCommand(new ManipulatorCommand(manipulatorSubsystem, rightJoystick));
        // Configure the trigger bindings
        configureBindings();
    }

    // Configure Button Bindings
    private void configureBindings() {
        climbButton.whileTrue(new ClimbMotorCommand(climbMotorSubsystem, Constants.ClimbConstants.CLIMB_SPEED));
        climbButtonReverse.whileTrue(new ClimbMotorCommand(climbMotorSubsystem, -Constants.ClimbConstants.CLIMB_SPEED));

        elevatorPositionOne.whileTrue(new ElevatorSetPositionsCommand(m_elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SPEED, 0));
        elevatorPositionTwo.whileTrue(new ElevatorSetPositionsCommand(m_elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SPEED, 1));
        elevatorPositionThree.whileTrue(new ElevatorSetPositionsCommand(m_elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SPEED, 2));
        elevatorPositionFour.whileTrue(new ElevatorSetPositionsCommand(m_elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SPEED, 3));

        // manipulatorRotateButton.whileTrue(new
        // ManipulatorPositionOne(manipulatorSubsystem));
        manipulatorRotateButton.toggleOnTrue(new ManipulatorRotateCommand(rotateManipulatorSubsystem));
        // manipulatorRotateButton.toggleOnFalse(new ManipulatorRotateCommand(rotateManipulatorSubsystem));
        intakeButton.whileTrue(new IntakeCommand(intakeSubsystem, ManipulatorConstants.INTAKE_MOTOR_SPEED));
        outtakeButton.whileTrue(new IntakeCommand(intakeSubsystem, -ManipulatorConstants.INTAKE_MOTOR_SPEED));
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
