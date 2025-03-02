// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.SendableChooserSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimbMotorCommand;
import frc.robot.subsystems.ClimbMotorSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManipulatorCommand;
import frc.robot.commands.ManipulatorPositionOne;
import frc.robot.commands.ManipulatorRotateCommand;
import frc.robot.commands.ResetToAbsolutes;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.RotateManipulatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorSteadyCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveTeleopDrive;
import frc.robot.commands.ZeroGyro;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Swerve.AutoContainer;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.Constants.ManipulatorConstants;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //Subsytems
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
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

    private JoystickButton manipulatorRotateButton = new JoystickButton(rightJoystick, ManipulatorConstants.MANIPULATOR_ROTATE_BUTTON);
    private JoystickButton intakeButton = new JoystickButton(rightJoystick, Constants.INTAKE_BUTTON);
    private JoystickButton outtakeButton = new JoystickButton(rightJoystick, Constants.OUTTAKE_BUTTON);
    private JoystickButton resetGyroButton = new JoystickButton(leftJoystick, Constants.RESET_GYRO_BUTTON);
    private JoystickButton resetAbsoluteButton = new JoystickButton(leftJoystick, Constants.REsET_ABSOLUTE_BUTTON);

    private SendableChooser autoChooser = new SendableChooser();
    private AutoContainer autoContainer = new AutoContainer();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CameraServer.startAutomaticCapture();

    Shuffleboard.getTab("Autos").add(autoChooser);
    // Configure the trigger bindings
    swerveSubsystem.setDefaultCommand(new SwerveTeleopDrive(
      swerveSubsystem, 
      () -> leftJoystick.getX(), 
      () -> leftJoystick.getY(), 
      () -> middleJoystick.getX(), 
      () -> true));
      manipulatorSubsystem.setDefaultCommand(new ManipulatorCommand(manipulatorSubsystem, rightJoystick));
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
    
        climbButton.whileTrue(new ClimbMotorCommand(climbMotorSubsystem, Constants.ClimbConstants.CLIMB_SPEED));
        climbButtonReverse.whileTrue(new ClimbMotorCommand(climbMotorSubsystem, -Constants.ClimbConstants.CLIMB_SPEED));

        elevatorUpButton.whileTrue(new ElevatorCommand(m_elevatorSubsystem, -Constants.ElevatorConstants.ELEVATOR_SPEED));
        elevatorDownButton.whileTrue(new ElevatorCommand(m_elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SPEED));
        elevatorUpButton.whileFalse(new ElevatorSteadyCommand(m_elevatorSubsystem));
        elevatorDownButton.whileFalse(new ElevatorSteadyCommand(m_elevatorSubsystem));
        
        resetGyroButton.whileTrue(new ZeroGyro(swerveSubsystem));
        resetAbsoluteButton.whileTrue(new ResetToAbsolutes(swerveSubsystem));

        // manipulatorRotateButton.whileTrue(new
        // ManipulatorPositionOne(manipulatorSubsystem));
        manipulatorRotateButton.toggleOnTrue(new ManipulatorRotateCommand(rotateManipulatorSubsystem));
        // manipulatorRotateButton.toggleOnFalse(new ManipulatorRotateCommand(rotateManipulatorSubsystem));
        intakeButton.whileTrue(new IntakeCommand(intakeSubsystem, ManipulatorConstants.INTAKE_MOTOR_SPEED));
        outtakeButton.whileTrue(new IntakeCommand(intakeSubsystem, -ManipulatorConstants.INTAKE_MOTOR_SPEED * 0.25));
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
