package frc.robot.commands.AutoCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class RecordingDrive extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private Supplier<Double> xSupplier, ySupplier, zSupplier;
    private Supplier<Boolean> slowButton;
    private Supplier<Boolean> fieldOrientedSupplier;

    private Supplier<Boolean> recordInputSupplier;
    private Supplier<Joystick> buttonJoystick;
    private Supplier<JoystickButton> elevatorPositionOne;
    private Supplier<JoystickButton> elevatorPositionTwo;
    private Supplier<JoystickButton> elevatorPositionThree;
    private Supplier<JoystickButton> elevatorPositionFour;

    private Supplier<JoystickButton> intakeButton;
    private Supplier<JoystickButton> outtakeButton;




    public RecordingDrive(SwerveSubsystem swerveSubsystem, Supplier<Double> xSupplier, Supplier<Double> ySupplier,
            Supplier<Double> zSupplier, 
            Supplier<Boolean> slowButton,
            Supplier<Boolean> fieldOrientedSupplier, 
            Supplier<Boolean> recordInputSupplier, 
            Supplier<JoystickButton> elevatorPositionOne,
            Supplier<JoystickButton> elevatorPositionTwo,
            Supplier<JoystickButton> elevatorPositionThree,
            Supplier<JoystickButton> elevatorPositionFour,
            Supplier<Joystick> buttonJoystick,
            Supplier<JoystickButton> intakeButton,
            Supplier<JoystickButton> outtakeButton) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.zSupplier = zSupplier;
        this.slowButton = slowButton;
        this.fieldOrientedSupplier = fieldOrientedSupplier;
        this.recordInputSupplier = recordInputSupplier;
        this.buttonJoystick = buttonJoystick;
        this.elevatorPositionOne = elevatorPositionOne;
        this.elevatorPositionTwo = elevatorPositionTwo;
        this.elevatorPositionThree = elevatorPositionThree;
        this.elevatorPositionFour = elevatorPositionFour;
        this.intakeButton = intakeButton;
        this.outtakeButton = outtakeButton;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.setSwerveModuleStates(new SwerveModuleState[]{
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0)),
        });
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
        if (recordInputSupplier.get()) {
            System.out.println("Recording");
            swerveSubsystem.recordInput(-xSpeed / 2, -ySpeed / 2, -zSpeed / 2, buttonJoystick.get().getY(), elevatorPositionOne.get().getAsBoolean(), elevatorPositionTwo.get().getAsBoolean(), elevatorPositionThree.get().getAsBoolean(), elevatorPositionFour.get().getAsBoolean(), intakeButton.get().getAsBoolean(), outtakeButton.get().getAsBoolean());
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
