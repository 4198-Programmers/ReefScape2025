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
    private Supplier<Boolean> fieldOrientedSupplier;

    public SwerveTeleopDrive(SwerveSubsystem swerveSubsystem, Supplier<Double> xSupplier, Supplier<Double> ySupplier,
            Supplier<Double> zSupplier, Supplier<Boolean> fieldOrientedSupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.zSupplier = zSupplier;
        this.fieldOrientedSupplier = fieldOrientedSupplier;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        double xSpeed = deadband(xSupplier.get(), Constants.DEADBAND);
        double ySpeed = deadband(ySupplier.get(), Constants.DEADBAND);
        double zSpeed = deadband(zSupplier.get(), Constants.DEADBAND);

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

    private double deadband(double value,  double threshold){
        
        if(Math.abs(value) < threshold){
            return 0;
        }
        return value;
    }
}
