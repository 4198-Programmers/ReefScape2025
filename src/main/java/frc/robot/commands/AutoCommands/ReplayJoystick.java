package frc.robot.commands.AutoCommands;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManipulatorCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.RotateManipulatorSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class ReplayJoystick extends Command {
    private SwerveSubsystem swerveSubsystem;
    private ManipulatorSubsystem manipulatorSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private RotateManipulatorSubsystem rotateManipulatorSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private List<String[]> recordedInputs = new ArrayList<>();
    private int index = 0;
    String[] data;
    
    public ReplayJoystick(
        SwerveSubsystem swerveSubsystem, 
        ManipulatorSubsystem manipulatorSubsystem, 
        ElevatorSubsystem elevatorSubsystem, 
        RotateManipulatorSubsystem rotateManipulatorSubsystem,
        IntakeSubsystem intakeSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.manipulatorSubsystem = manipulatorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.rotateManipulatorSubsystem = rotateManipulatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(swerveSubsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        String directory = Filesystem.getDeployDirectory().toString();

        try (BufferedReader reader = new BufferedReader(new FileReader(String.format("%s/recordedAutos/place.csv", directory)))) {
            String line = reader.readLine();
            while ((line = reader.readLine()) != null) {
                recordedInputs.add(line.split(","));
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (index < recordedInputs.size()) {
            data = recordedInputs.get(index);
            new ScheduleCommand(new ManipulatorCommand(manipulatorSubsystem, new Joystick(2), Double.parseDouble(data[3]))).schedule();
            
            intake(data[8].replaceAll("\\s+", "").equals("true"), data[9].replaceAll("\\s+", "").equals("true"), intakeSubsystem);
            // manipulatorMove(Double.parseDouble(data[3]), manipulatorSubsystem);
            moveElevator(Boolean.parseBoolean(data[4].replaceAll("\\s+", "")), Boolean.parseBoolean(data[5].replaceAll("\\s+", "")), Boolean.parseBoolean(data[6].replaceAll("\\s+", "")), Boolean.parseBoolean(data[7].replaceAll("\\s+", "")), elevatorSubsystem, rotateManipulatorSubsystem);
            swerveSubsystem.drive(Double.parseDouble(data[0]), Double.parseDouble(data[1]), Double.parseDouble(data[2]), true);

            // if (data[4] == "true") {

            // }
            // System.out.println(String.format("\"%s\"", data[4].replaceAll("\\s+", "")));
            index++;
        } else {
            swerveSubsystem.drive(0, 0, 0, false);
            manipulatorSubsystem.turnPrimaryJoint(0);
            intakeSubsystem.stop();
            end(true);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public void manipulatorMove(double yValue, ManipulatorSubsystem manipulatorSubsystem) {
        yValue = 0.5;

        // System.out.println(yValue);
        if(yValue > 0.1 || yValue < -0.1) {
            // System.out.println("Turn Primary Joint: " + yValue);
            manipulatorSubsystem.turnPrimaryJoint(-yValue * Constants.ManipulatorConstants.MANIPULATOR_MOTOR_SPEED);
        } else {
        manipulatorSubsystem.turnPrimaryJoint(0);
        }
    }

    public void moveElevator(boolean elevatorOneButton, boolean elevatorTwoButton, boolean elevatorThreeButton, boolean elevatorFourButton,
    ElevatorSubsystem elevatorSubsystem, RotateManipulatorSubsystem rotateManipulatorSubsystem) {
        int elevatorPosition = getElevatorPosition(elevatorOneButton, elevatorTwoButton, elevatorThreeButton, elevatorFourButton);
        // System.out.println(elevatorOneButton + ", " + elevatorTwoButton + ", " + elevatorThreeButton + ", " + elevatorFourButton);
        // System.out.println("Elevator Position: " + elevatorPosition);
        new ScheduleCommand(new ElevatorCommand(elevatorSubsystem, rotateManipulatorSubsystem, elevatorPosition)).schedule();
    }

    public int getElevatorPosition(boolean elevatorOneButton, boolean elevatorTwoButton, boolean elevatorThreeButton, boolean elevatorFourButton) {
        int elevatorPosition = 10;
        if (elevatorOneButton) {
            elevatorPosition = 0;
        } else if (elevatorTwoButton) {
            elevatorPosition = 1;
        } else if (elevatorThreeButton) {
            elevatorPosition = 2;
        } else if (elevatorFourButton) {
            elevatorPosition = 3;
        }
        return elevatorPosition;
    }

    public void intake(boolean intakeButton, boolean outtakeButton, IntakeSubsystem intakeSubsystem) {
        if (intakeButton) {
            new ScheduleCommand(new IntakeCommand(intakeSubsystem, Constants.ManipulatorConstants.INTAKE_MOTOR_SPEED)).schedule();
        } else if (outtakeButton) {
            new ScheduleCommand(new OuttakeCommand(intakeSubsystem, Constants.ManipulatorConstants.INTAKE_MOTOR_SPEED * -0.25)).schedule();
        } else {
            intakeSubsystem.stop();
        }
    }
    
}
