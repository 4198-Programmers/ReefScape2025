package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallIntakeSubsystem extends SubsystemBase {
    // Initialize ball intake motor
    private SparkMax ballIntakeMotor = new SparkMax(Constants.BALL_INTAKE_MOTOR_ID, MotorType.kBrushless);
    
    // Set motor speed
    public void setGrabberMotorSpeed(double speed) {
        ballIntakeMotor.set(speed);
    }

    // Stop motor
    public void stop() {
        ballIntakeMotor.set(0);
    }
}
