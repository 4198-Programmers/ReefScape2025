package frc.robot.commands.AutoCommands;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class ReplayJoystick extends Command {
    private SwerveSubsystem swerveSubsystem;
    private List<String[]> recordedInputs = new ArrayList<>();
    private int index = 0;
    String[] data;
    
    public ReplayJoystick(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        String directory = Filesystem.getDeployDirectory().toString();

        try (BufferedReader reader = new BufferedReader(new FileReader(String.format("%s/recordedAutos/rotateandturn.csv", directory)))) {
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
            swerveSubsystem.drive(Double.parseDouble(data[0]), Double.parseDouble(data[1]), Double.parseDouble(data[2]), true);
            index++;
        } else {
            swerveSubsystem.drive(0, 0, 0, false);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
