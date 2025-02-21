package frc.robot.commands;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double speed;

    public IntakeCommand(IntakeSubsystem subsystem, double speed) {
        intakeSubsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        System.out.println("ran intake");
        intakeSubsystem.runIntake(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.runIntake(0);
    }
    
}
