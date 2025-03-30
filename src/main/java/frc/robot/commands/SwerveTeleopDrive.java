package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class SwerveTeleopDrive extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private Supplier<Double> xSupplier, ySupplier, zSupplier;
    private Supplier<Boolean> fieldOrientedSupplier, slowButton;


    public SwerveTeleopDrive(SwerveSubsystem swerveSubsystem, Supplier<Double> xSupplier, Supplier<Double> ySupplier,
            Supplier<Double> zSupplier, Supplier<Boolean> slowButton, Supplier<Boolean> fieldOrientedSupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.zSupplier = zSupplier;
        this.slowButton = slowButton;
        this.fieldOrientedSupplier = fieldOrientedSupplier;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        double xSpeed = deadband(xSupplier.get(), Constants.DEADBAND);
        double ySpeed = deadband(ySupplier.get(), Constants.DEADBAND);
        double zSpeed = deadband(zSupplier.get(), Constants.DEADBAND);

        if (slowButton.get()) {
            xSpeed = xSpeed / 2;
            ySpeed = ySpeed / 2;
            zSpeed = zSpeed / 2;
        }

        // System.out.println("xSpeed:" + xSpeed + " ySpeed: " + ySpeed + " zSpeed: " + zSpeed);

        //swerveSubsystem.getModuleAngles();
        swerveSubsystem.drive(-xSpeed / 2, -ySpeed / 2, -zSpeed / 2, fieldOrientedSupplier.get());
        int index = 0;

        // for (SwerveModuleState sModuleState : swerveSubsystem.getSwerveModuleStates()) {

        //     System.out.println("index:" + index++ + " angle: " + sModuleState.angle + " spd: " + sModuleState.speedMetersPerSecond);

        // }

        // swerveSubsystem.getModuleAngles();
        // System.out.println("ran");
        
    }

    /**
     * This method is used to apply a deadband to the joystick values
     * @param value
     * @param threshold
     * @return the value after applying the deadband
     */
    private double deadband(double value,  double threshold){
        
        if(Math.abs(value) < threshold){
            return 0;
        }
        return value;
    }
}
