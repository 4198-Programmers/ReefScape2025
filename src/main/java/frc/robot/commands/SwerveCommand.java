package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private Supplier<Double> xSpeed, ySpeed, rotationSpeed;
    private Supplier<Boolean> slowButton;
    private boolean fieldOriented;

    public SwerveCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotationSpeed, Supplier<Boolean> slowButton, boolean fieldOriented) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotationSpeed = rotationSpeed;
        this.slowButton = slowButton;
        this.fieldOriented = fieldOriented;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        double xSpeedValue = deadband(xSpeed.get(), Constants.DEADBAND);
        double ySpeedValue = deadband(ySpeed.get(), Constants.DEADBAND);
        double rotationSpeedValue = deadband(rotationSpeed.get(), Constants.DEADBAND);
        if (slowButton.get()) {
            xSpeedValue = xSpeedValue / 2;
            ySpeedValue = ySpeedValue / 2;
            rotationSpeedValue = rotationSpeedValue / 2;
        }
        swerveSubsystem.drive(xSpeedValue, -ySpeedValue, rotationSpeedValue, fieldOriented);
    }
        

    private double deadband(double value,  double threshold){
        
        if(Math.abs(value) < threshold){
            return 0;
        }
        return value;
    }

}
    
