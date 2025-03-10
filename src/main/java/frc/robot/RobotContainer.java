// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.SendableChooserSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Autos;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.ClimbMotorCommand;
import frc.robot.subsystems.AutoContainer;
import frc.robot.subsystems.ClimbMotorSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManipulatorCommand;
import frc.robot.commands.PhotonVisionCommand;
import frc.robot.commands.ManipulatorToPoint;
import frc.robot.commands.ResetToAbsolutes;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.RotateManipulatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveTeleopDrive;
import frc.robot.commands.ZeroGyro;
import frc.robot.subsystems.ExampleSubsystem;
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
    private final PoseEstimatorSubsystem poseEstimatorSubsystem = new PoseEstimatorSubsystem(swerveSubsystem, Constants.PHOTON_CAMERA);

    private final PhotonSubsystem photonSubsystem = new PhotonSubsystem();
    private final AutoContainer autoContainer = new AutoContainer(swerveSubsystem, manipulatorSubsystem, rotateManipulatorSubsystem, intakeSubsystem, m_elevatorSubsystem);

    // Subsystems
    private final ClimbMotorSubsystem climbMotorSubsystem = new ClimbMotorSubsystem();

    // Joysticks
    private final Joystick leftJoystick = new Joystick(Constants.JOYSTICK_LEFT_ID);
    private final Joystick middleJoystick = new Joystick(Constants.JOYSTICK_MIDDLE_ID);
    private final Joystick rightJoystick = new Joystick(Constants.JOYSTICK_RIGHT_ID);

    // Buttons
    private final JoystickButton climbButton = new JoystickButton(rightJoystick, Constants.ClimbConstants.CLIMB_FORWARD_BUTTON);
    private final JoystickButton climbButtonReverse = new JoystickButton(rightJoystick, Constants.ClimbConstants.CLIMB_REVERSE_BUTTON);

    // private final JoystickButton elevatorUpButton = new JoystickButton(rightJoystick, Constants.ElevatorConstants.ELEVATOR_UP_BUTTON);
    // private final JoystickButton elevatorDownButton = new JoystickButton(rightJoystick, Constants.ElevatorConstants.ELEVATOR_DOWN_BUTTON);

    private final JoystickButton elevatorPositionOne = new JoystickButton(rightJoystick, Constants.ElevatorConstants.ELEVATOR_BUTTON_POSITION_ONE);
    private final JoystickButton elevatorPositionTwo = new JoystickButton(rightJoystick, Constants.ElevatorConstants.ELEVATOR_BUTTON_POSITION_TWO);
    private final JoystickButton elevatorPositionThree = new JoystickButton(rightJoystick, Constants.ElevatorConstants.ELEVATOR_BUTTON_POSITION_THREE);
    private final JoystickButton elevatorPositionFour = new JoystickButton(rightJoystick, Constants.ElevatorConstants.ELEVATOR_BUTTON_POSITION_FOUR);

    private JoystickButton manipulatorRotateButton = new JoystickButton(rightJoystick, ManipulatorConstants.MANIPULATOR_ROTATE_BUTTON);
    private JoystickButton intakeButton = new JoystickButton(rightJoystick, Constants.INTAKE_BUTTON);
    private JoystickButton outtakeButton = new JoystickButton(rightJoystick, Constants.OUTTAKE_BUTTON);
    private JoystickButton resetGyroButton = new JoystickButton(leftJoystick, Constants.RESET_GYRO_BUTTON);
    private JoystickButton resetAbsoluteButton = new JoystickButton(leftJoystick, Constants.REsET_ABSOLUTE_BUTTON);

    private JoystickButton photonAlignLeftButton = new JoystickButton(middleJoystick, 3);
    private JoystickButton photonAlignRightButton = new JoystickButton(middleJoystick, 4);
    private JoystickButton zeroManipulator = new JoystickButton(middleJoystick, 6);
    private JoystickButton getManipulator = new JoystickButton(middleJoystick, 4);

    private JoystickButton setManipulatorToPointOne = new JoystickButton(leftJoystick, 6);
    private JoystickButton setManipulatorToPointTwo = new JoystickButton(leftJoystick, 4);
    private JoystickButton zeroManipulatorRotate = new JoystickButton(rightJoystick, 6);

    // SendableChooser<Command> autoChooser = new SendableChooser<>();
    SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // CameraServer.startAutomaticCapture();
    // Configure the trigger bindings
    autoContainer.SetupAutoOptions(autoChooser);
    swerveSubsystem.setDefaultCommand(new SwerveTeleopDrive(
      swerveSubsystem, 
      () -> leftJoystick.getX(), 
      () -> leftJoystick.getY(), 
      () -> middleJoystick.getX(), 
      () -> true));
      manipulatorSubsystem.setDefaultCommand(new ManipulatorCommand(manipulatorSubsystem, rightJoystick));
    configureBindings();
    System.out.println(autoChooser.toString());
    System.out.println(autoChooser.getSelected());
    SmartDashboard.putData(autoChooser);
    Shuffleboard.getTab("Autos").add(autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0,0).withSize(3, 1);
    // .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0,0).withSize(3, 1);

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
        // elevatorUpButton.whileTrue(new ElevatorCommand(m_elevatorSubsystem, -Constants.ElevatorConstants.ELEVATOR_SPEED));
        // elevatorDownButton.whileTrue(new ElevatorCommand(m_elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SPEED));

        elevatorPositionOne.whileTrue(new ElevatorCommand(m_elevatorSubsystem, rotateManipulatorSubsystem, 0));
        elevatorPositionTwo.whileTrue(new ElevatorCommand(m_elevatorSubsystem, rotateManipulatorSubsystem, 1));
        elevatorPositionThree.whileTrue(new ElevatorCommand(m_elevatorSubsystem, rotateManipulatorSubsystem, 2));
        elevatorPositionFour.whileTrue(new ElevatorCommand(m_elevatorSubsystem, rotateManipulatorSubsystem, 3));
        
        resetGyroButton.whileTrue(new ZeroGyro(swerveSubsystem, poseEstimatorSubsystem));
        resetAbsoluteButton.whileTrue(new ResetToAbsolutes(swerveSubsystem));

        // manipulatorRotateButton.whileTrue(new
        // ManipulatorPositionOne(manipulatorSubsystem));
        manipulatorRotateButton.onTrue(rotateManipulatorSubsystem.RotateManipulatorCommand());
        
        // manipulatorRotateButton.toggleOnFalse(new ManipulatorRotateCommand(rotateManipulatorSubsystem));
        intakeButton.whileTrue(new IntakeCommand(intakeSubsystem, ManipulatorConstants.INTAKE_MOTOR_SPEED));
        outtakeButton.whileTrue(new IntakeCommand(intakeSubsystem, -ManipulatorConstants.INTAKE_MOTOR_SPEED * 0.25));
        photonAlignLeftButton.toggleOnTrue(new ChaseTagCommand(Constants.PHOTON_CAMERA, swerveSubsystem, () -> swerveSubsystem.getPose(), Constants.AprilTagConstants.APRILTAG_LEFT));
        photonAlignRightButton.toggleOnTrue(new ChaseTagCommand(Constants.PHOTON_CAMERA, swerveSubsystem, () -> swerveSubsystem.getPose(), Constants.AprilTagConstants.APRILTAG_RIGHT));
        // photonVisionButton.onTrue(poseEstimatorSubsystem.ResetPoseEstimator());
        zeroManipulator.whileTrue(manipulatorSubsystem.ZeroManipulatorCommand());
        getManipulator.whileTrue(manipulatorSubsystem.GetManipulatorEncoder());
        setManipulatorToPointOne.toggleOnTrue(new ManipulatorToPoint(manipulatorSubsystem, 0));
        setManipulatorToPointTwo.toggleOnTrue(new ManipulatorToPoint(manipulatorSubsystem, 1));

        zeroManipulatorRotate.onTrue(rotateManipulatorSubsystem.ZeroManipulatorRotate());
  }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.

     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        
        return autoChooser.getSelected();
    }
}
