package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TankSubsystem extends SubsystemBase {
    /*
     * The first portion of any code is used to define items
     * a CANSparkMax is a type of motor
     * 
     */

    // telling the code what it can use, Motor Id
    // the "new" in the statment creates a new item or thing for the bot
    private SparkMax frontLeftMotor = new SparkMax(Constants.FRONT_LEFT_MOTOR, MotorType.kBrushless);
    private SparkMax frontRightMotor = new SparkMax(Constants.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
    private SparkMax backLeftMotor = new SparkMax(Constants.BACK_LEFT_MOTOR, MotorType.kBrushless);
    private SparkMax backRightMotor = new SparkMax(Constants.BACK_RIGHT_MOTOR, MotorType.kBrushless);

    // tells the bot how many time a motor has spun
    private RelativeEncoder frontLeftEncoder = frontLeftMotor.getEncoder();
    private RelativeEncoder frontRightEncoder = frontRightMotor.getEncoder();
    private RelativeEncoder backLeftEncoder = backLeftMotor.getEncoder();
    private RelativeEncoder backRightEncoder = backRightMotor.getEncoder();

    DifferentialDrive tankDrive = new DifferentialDrive (
        (double leftOutput) -> {
            frontLeftMotor.set(leftOutput);
            backLeftMotor.set(leftOutput);
        }, 
        (double rightOutput) -> {
            frontRightMotor.set(rightOutput);
            backRightMotor.set(rightOutput);
        }
    );
    
    public void drive(double zRotate, double xAxis){
        tankDrive.arcadeDrive(Constants.DRIVE_SPEED * xAxis, Constants.DRIVE_SPEED * zRotate);
    }
}
