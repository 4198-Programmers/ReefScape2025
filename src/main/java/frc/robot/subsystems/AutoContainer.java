package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SwerveTeleopDrive;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class AutoContainer extends SubsystemBase{

    Shuffleboard shuffleboard;
    ShuffleboardTab autoTab;
    SwerveSubsystem swerveSubsystem;;
    ElevatorSubsystem elevatorSubsystem;
    IntakeSubsystem intakeSubsystem;
    ManipulatorSubsystem manipulatorSubsystem;
    RotateManipulatorSubsystem rotateManipulatorSubsystem;


    public AutoContainer(SwerveSubsystem swerveSubsystem, ManipulatorSubsystem manipulatorSubsystem, RotateManipulatorSubsystem rotateManipulatorSubsystem, IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        this.manipulatorSubsystem = manipulatorSubsystem;
        this.rotateManipulatorSubsystem = rotateManipulatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    public void SetupAutoOptions(SendableChooser<Command> sendableChooser){
        sendableChooser.setDefaultOption("PullForwardAuto", swerveSubsystem.getAutonomousCommand("PullForwardAuto"));
        sendableChooser.addOption("PullForwardAuto", swerveSubsystem.getAutonomousCommand("PullForwardAuto"));
        sendableChooser.addOption("ReadyToPlaceAuto", swerveSubsystem.getAutonomousCommand("ReadyToPlaceAuto"));
        sendableChooser.addOption("TestAuto", swerveSubsystem.getAutonomousCommand("TestAuto"));
        sendableChooser.addOption("EnemyOneToReefL4", swerveSubsystem.getAutonomousCommand("EnemyOneToReefL4"));
        sendableChooser.addOption("EnemyTwoToReefL4", swerveSubsystem.getAutonomousCommand("EnemyTwoToReefL4"));
        sendableChooser.addOption("EnemyThreeToReefL4", swerveSubsystem.getAutonomousCommand("EnemyThreeToReefL4"));
        sendableChooser.addOption("TeammateOneToReefL4", swerveSubsystem.getAutonomousCommand("TeammateOneToReefL4"));
        sendableChooser.addOption("TeammateTwoToReefL4", swerveSubsystem.getAutonomousCommand("TeammateTwoToReefL4"));
        sendableChooser.addOption("TeammateThreeToReefL4", swerveSubsystem.getAutonomousCommand("TeammateThreeToReefL4"));
        sendableChooser.addOption("EnemyTwoL3Human", swerveSubsystem.getAutonomousCommand("EnemyTwoL3Human"));
        sendableChooser.addOption("SideDump", swerveSubsystem.getAutonomousCommand("SideDump"));
        sendableChooser.addOption("CenterDump", swerveSubsystem.getAutonomousCommand("CenterDump"));
   
        sendableChooser.addOption("TestForward", new SwerveTeleopDrive(swerveSubsystem, () -> 0.0, () -> -0.2, () -> 0.0, () -> false, () -> false));
    }
    
    
}
