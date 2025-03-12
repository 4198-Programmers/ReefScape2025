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
        sendableChooser.addOption("TestForward", new SwerveTeleopDrive(swerveSubsystem, () -> 0.0, () -> 0.2, () -> 0.0, () -> false));
    }
    
    
}
