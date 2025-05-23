// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

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
import frc.robot.commands.GeneralElevatorCommand;
import frc.robot.commands.GyroSetAngle90;
import frc.robot.commands.GyroSetAngleNeg90;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveTeleopDrive;
import frc.robot.commands.TrajectoryFollowing;
import frc.robot.commands.ZeroGyro;
import frc.robot.commands.AutoCommands.AutoResetOdometry;
import frc.robot.commands.AutoCommands.RecordingDrive;
import frc.robot.commands.AutoCommands.ReplayJoystick;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.Constants.ManipulatorConstants;

public class RobotContainer {    
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

    private final JoystickButton slowDriveButton = new JoystickButton(middleJoystick, 2);
    // Buttons
    private final JoystickButton climbButton = new JoystickButton(rightJoystick, Constants.ClimbConstants.CLIMB_FORWARD_BUTTON);
    private final JoystickButton climbButtonReverse = new JoystickButton(rightJoystick, Constants.ClimbConstants.CLIMB_REVERSE_BUTTON);

    private final JoystickButton elevatorPositionOne = new JoystickButton(rightJoystick, Constants.ElevatorConstants.ELEVATOR_BUTTON_POSITION_ONE);
    private final JoystickButton elevatorPositionTwo = new JoystickButton(rightJoystick, Constants.ElevatorConstants.ELEVATOR_BUTTON_POSITION_TWO);
    private final JoystickButton elevatorPositionThree = new JoystickButton(rightJoystick, Constants.ElevatorConstants.ELEVATOR_BUTTON_POSITION_THREE);
    private final JoystickButton elevatorPositionFour = new JoystickButton(rightJoystick, Constants.ElevatorConstants.ELEVATOR_BUTTON_POSITION_FOUR);

    private JoystickButton manipulatorRotateButton = new JoystickButton(rightJoystick, ManipulatorConstants.MANIPULATOR_ROTATE_BUTTON);
    private JoystickButton intakeButton = new JoystickButton(rightJoystick, Constants.INTAKE_BUTTON);
    private JoystickButton outtakeButton = new JoystickButton(rightJoystick, Constants.OUTTAKE_BUTTON);
    private JoystickButton fasterOuttakeButton = new JoystickButton(rightJoystick, Constants.FASTER_OUTTAKE_BUTTON);
    private JoystickButton resetGyroButton = new JoystickButton(leftJoystick, Constants.RESET_GYRO_BUTTON);
    private JoystickButton resetAbsoluteButton = new JoystickButton(leftJoystick, Constants.REsET_ABSOLUTE_BUTTON);

    private JoystickButton photonAlignLeftButton = new JoystickButton(middleJoystick, 3);
    private JoystickButton photonAlignRightButton = new JoystickButton(middleJoystick, 4);
    private JoystickButton zeroManipulator = new JoystickButton(middleJoystick, 6);

    private JoystickButton moveElevatorUpButton = new JoystickButton(rightJoystick, 6);
    private JoystickButton moveElevatorDownButton = new JoystickButton(rightJoystick, 4);

    private JoystickButton recordInputs = new JoystickButton(middleJoystick, 1);
    private JoystickButton replayInputs = new JoystickButton(middleJoystick, 5);
    private JoystickButton trajectoryJoystickButton = new JoystickButton(leftJoystick, 1); // trigger

    // SendableChooser<Command> autoChooser = new SendableChooser<>();
    SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // CameraServer.startAutomaticCapture();
    // Configure the trigger bindings
    NamedCommands.registerCommand("L4ReefPlace", new ManipulatorToPoint(manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, 8).withTimeout(1));
    NamedCommands.registerCommand("L3ReefPlace", new ManipulatorToPoint(manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, 2));
    NamedCommands.registerCommand("L2ReefPlace", new ManipulatorToPoint(manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, 1));
    NamedCommands.registerCommand("HumanPlayer", new ManipulatorToPoint(manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, 0));
    NamedCommands.registerCommand("OuttakeCommand", new OuttakeCommand(intakeSubsystem, -0.30).withTimeout(2));
    NamedCommands.registerCommand("ZeroGyro", new ZeroGyro(swerveSubsystem, poseEstimatorSubsystem).withTimeout(0.1));
    NamedCommands.registerCommand("L1Reef", new ManipulatorToPoint(manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, 4).withTimeout(1));
    NamedCommands.registerCommand("AlignCenterAprilTag", new ChaseTagCommand(Constants.PHOTON_CAMERA, swerveSubsystem, () -> swerveSubsystem.getPose(), Constants.AprilTagConstants.APRILTAG_MIDDLE).withTimeout(5));
    NamedCommands.registerCommand("ResetTo1MeterAway", new AutoResetOdometry(swerveSubsystem, 6.8, 4.007, 180));
    NamedCommands.registerCommand("ManipulatorDown", new ManipulatorCommand(manipulatorSubsystem, leftJoystick, -0.2).withTimeout(1));
    NamedCommands.registerCommand("RobotForwardDrive", new SwerveTeleopDrive(swerveSubsystem, () -> 0.0, () -> -0.4, () -> 0.0, () -> false, () -> false).withTimeout(1));
    NamedCommands.registerCommand("RobotBackDrive", new SwerveTeleopDrive(swerveSubsystem, () -> 0.0, () -> 0.4, () -> 0.0, () -> false, () -> false).withTimeout(0.5));
    NamedCommands.registerCommand("FollowLeftPath", new TrajectoryFollowing(swerveSubsystem).withTimeout(3));
    autoContainer.SetupAutoOptions(autoChooser);

    if (DriverStation.isTest()) { // if it's in test mode, it does a custom drive command to record
      swerveSubsystem.setDefaultCommand(new RecordingDrive(
        swerveSubsystem, 
        () -> leftJoystick.getX(),
        () -> leftJoystick.getY(), 
        () -> middleJoystick.getX(), 
        () -> slowDriveButton.getAsBoolean(),
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
        () -> leftJoystick.getZ(), 
        () -> slowDriveButton.getAsBoolean(),
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

        elevatorPositionOne.whileTrue(new ManipulatorToPoint(manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, 0)); //Human Player Height
        elevatorPositionTwo.whileTrue(new ManipulatorToPoint(manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, 1)); //Level 2
        elevatorPositionThree.whileTrue(new ManipulatorToPoint(manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, 2)); //Level 3
        elevatorPositionFour.whileTrue(new ManipulatorToPoint(manipulatorSubsystem, elevatorSubsystem, rotateManipulatorSubsystem, 3)); //Level 4
        
        resetGyroButton.whileTrue(new ZeroGyro(swerveSubsystem, poseEstimatorSubsystem));
        resetAbsoluteButton.whileTrue(new ResetToAbsolutes(swerveSubsystem));

        manipulatorRotateButton.onTrue(rotateManipulatorSubsystem.RotateManipulatorCommand());
        zeroManipulator.whileTrue(manipulatorSubsystem.ZeroManipulatorCommand());
        
        intakeButton.whileTrue(new IntakeCommand(intakeSubsystem, ManipulatorConstants.INTAKE_MOTOR_SPEED));
        outtakeButton.whileTrue(new OuttakeCommand(intakeSubsystem, -ManipulatorConstants.INTAKE_MOTOR_SPEED * 0.25));
        fasterOuttakeButton.whileTrue(new OuttakeCommand(intakeSubsystem, -ManipulatorConstants.INTAKE_MOTOR_SPEED * 0.5));
        photonAlignLeftButton.toggleOnTrue(new ChaseTagCommand(Constants.PHOTON_CAMERA, swerveSubsystem, () -> swerveSubsystem.getPose(), Constants.AprilTagConstants.APRILTAG_LEFT));
        photonAlignRightButton.toggleOnTrue(new ChaseTagCommand(Constants.PHOTON_CAMERA, swerveSubsystem, () -> swerveSubsystem.getPose(), Constants.AprilTagConstants.APRILTAG_MIDDLE));
        
        moveElevatorUpButton.whileTrue(new GeneralElevatorCommand(elevatorSubsystem, true));
        moveElevatorDownButton.whileTrue(new GeneralElevatorCommand(elevatorSubsystem, false));
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
