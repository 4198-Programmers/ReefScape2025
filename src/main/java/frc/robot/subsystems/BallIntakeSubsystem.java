package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallIntakeSubsystem extends SubsystemBase {
    
    private SparkMax ballGrabberMotor = new SparkMax(Constants.BALL_GRABBER_MOTOR_ID, MotorType.kBrushless);
    
    public void setGrabberMotorSpeed(double speed) {
        ballGrabberMotor.set(speed);
    }

    public void stop() {
        ballGrabberMotor.set(0);
    }
}
