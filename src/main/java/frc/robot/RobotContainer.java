// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.commands.ManipulatorRotateCommand;
import frc.robot.commands.PhotonVisionCommand;
import frc.robot.commands.ManipulatorToPoint;
import frc.robot.commands.OuttakeCommand;
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
import frc.robot.commands.GyroSetAngle90;
import frc.robot.commands.GyroSetAngleNeg90;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveTeleopDrive;
import frc.robot.commands.ZeroGyro;
import frc.robot.commands.AutoCommands.RecordingDrive;
import frc.robot.commands.AutoCommands.ReplayJoystick;
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
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final PoseEstimatorSubsystem poseEstimatorSubsystem = new PoseEstimatorSubsystem(swerveSubsystem, Constants.PHOTON_CAMERA);

    private final PhotonSubsystem photonSubsystem = new PhotonSubsystem();
    private final AutoContainer autoContainer = new AutoContainer(swerveSubsystem, manipulatorSubsystem, rotateManipulatorSubsystem, intakeSubsystem, elevatorSubsystem);

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

    private JoystickButton moveManipulatorClockwise = new JoystickButton(rightJoystick, 6);
    private JoystickButton moveManipulatorCounterClockwise = new JoystickButton(rightJoystick, 4);

    // private JoystickButton setManipulatorToPointOne = new JoystickButton(leftJoystick, 6);
    // private JoystickButton setManipulatorToPointTwo = new JoystickButton(leftJoystick, 4);
    // private JoystickButton zeroManipulatorRotate = new JoystickButton(rightJoystick, 6);

    private JoystickButton recordInputs = new JoystickButton(middleJoystick, 1);
    private JoystickButton replayInputs = new JoystickButton(middleJoystick, 5);

    // SendableChooser<Command> autoChooser = new SendableChooser<>();
    SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // CameraServer.startAutomaticCapture();
    // Configure the trigger bindings
    NamedCommands.registerCommand("L4ReefPlace", new ManipulatorToPoint(manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, 3));
    NamedCommands.registerCommand("L3ReefPlace", new ManipulatorToPoint(manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, 2));
    NamedCommands.registerCommand("L2ReefPlace", new ManipulatorToPoint(manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, 1));
    NamedCommands.registerCommand("HumanPlayer", new ManipulatorToPoint(manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, 0));
    NamedCommands.registerCommand("OuttakeCommand", new OuttakeCommand(intakeSubsystem, -0.25).withTimeout(1));
    NamedCommands.registerCommand("ZeroGyro", new ZeroGyro(swerveSubsystem, poseEstimatorSubsystem).withTimeout(0.1));
    NamedCommands.registerCommand("L1Reef", new ManipulatorToPoint(manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, 4).withTimeout(1));
    NamedCommands.registerCommand("AlignCenterAprilTag", new ChaseTagCommand(Constants.PHOTON_CAMERA, swerveSubsystem, () -> swerveSubsystem.getPose(), Constants.AprilTagConstants.APRILTAG_MIDDLE).withTimeout(5));

    autoContainer.SetupAutoOptions(autoChooser);

    if (DriverStation.isTest()) {
      swerveSubsystem.setDefaultCommand(new RecordingDrive(
        swerveSubsystem, 
        () -> leftJoystick.getX(),
        () -> leftJoystick.getY(), 
        () -> middleJoystick.getX(), 
        () -> true,
        () -> recordInputs.getAsBoolean(),
        () -> elevatorPositionOne, 
        () -> elevatorPositionTwo, 
        () -> elevatorPositionThree, 
        () -> elevatorPositionFour, 
        () -> rightJoystick,
        () -> intakeButton,
        () -> outtakeButton));
    } else {
      swerveSubsystem.setDefaultCommand(new SwerveTeleopDrive(
        swerveSubsystem, 
        () -> leftJoystick.getX(),
        () -> leftJoystick.getY(), 
        () -> middleJoystick.getX(), 
        () -> true));
    }

    configureBindings();
    System.out.println(autoChooser.toString());
    System.out.println(autoChooser.getSelected());
    SmartDashboard.putData(autoChooser);
    PortForwarder.add(5800, "photonvision.local", 5800);
    // Shuffleboard.getTab("Autos").add(autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0,0).withSize(3, 1);
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
        // elevatorUpButton.whileTrue(new ElevatorCommand(elevatorSubsystem, -Constants.ElevatorConstants.ELEVATOR_SPEED));
        // elevatorDownButton.whileTrue(new ElevatorCommand(elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SPEED));

        elevatorPositionOne.whileTrue(new ManipulatorToPoint(manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, 0)); //Human Player Height
        elevatorPositionTwo.whileTrue(new ManipulatorToPoint(manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, 1)); //Level 2
        elevatorPositionThree.whileTrue(new ManipulatorToPoint(manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, 2)); //Level 3
        elevatorPositionFour.whileTrue(new ManipulatorToPoint(manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, 3)); //Level 4
        
        resetGyroButton.whileTrue(new ZeroGyro(swerveSubsystem, poseEstimatorSubsystem));
        resetAbsoluteButton.whileTrue(new ResetToAbsolutes(swerveSubsystem));

        // manipulatorRotateButton.whileTrue(new
        // ManipulatorPositionOne(manipulatorSubsystem));
        manipulatorRotateButton.onTrue(rotateManipulatorSubsystem.RotateManipulatorCommand());

        // logInputs.onTrue(swerveSubsystem.logInputsCommand());

        zeroManipulator.whileTrue(manipulatorSubsystem.ZeroManipulatorCommand());
        
        // manipulatorRotateButton.toggleOnFalse(new ManipulatorRotateCommand(rotateManipulatorSubsystem));
        intakeButton.whileTrue(new IntakeCommand(intakeSubsystem, ManipulatorConstants.INTAKE_MOTOR_SPEED));
        outtakeButton.whileTrue(new OuttakeCommand(intakeSubsystem, -ManipulatorConstants.INTAKE_MOTOR_SPEED * 0.25));
        photonAlignLeftButton.toggleOnTrue(new ChaseTagCommand(Constants.PHOTON_CAMERA, swerveSubsystem, () -> swerveSubsystem.getPose(), Constants.AprilTagConstants.APRILTAG_LEFT));
        photonAlignRightButton.toggleOnTrue(new ChaseTagCommand(Constants.PHOTON_CAMERA, swerveSubsystem, () -> swerveSubsystem.getPose(), Constants.AprilTagConstants.APRILTAG_MIDDLE));
        // photonVisionButton.onTrue(poseEstimatorSubsystem.ResetPoseEstimator());
        moveManipulatorClockwise.whileTrue(new ManipulatorRotateCommand(rotateManipulatorSubsystem, -0.05));
        moveManipulatorCounterClockwise.whileTrue(new ManipulatorRotateCommand(rotateManipulatorSubsystem, 0.05));

        replayInputs.whileTrue(new ReplayJoystick(swerveSubsystem, manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, intakeSubsystem));

  }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.

     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        
        return autoChooser.getSelected();
    }

    /**
     * Save the recorded joystick inputs to a file
     */
    public void saveRecording() {
        swerveSubsystem.logRecordedInputs();
    }

    public void assignManipulatorToStick() {
      manipulatorSubsystem.setDefaultCommand(new ManipulatorCommand(manipulatorSubsystem, rightJoystick, 0));
    }

}
