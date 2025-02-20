package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbMotorSubsystem extends SubsystemBase{
    
    private SparkMax climbMotor = new SparkMax(Constants.ClimbConstants.CLIMB_MOTOR_ID, MotorType.kBrushless);

    /**
     * Sets the climber motor to a given speed, positive is clockwise (the way we want to go to climb)
     */
    public void setClimbSpeed(double speed) {
        climbMotor.set(speed);
    }
    
    public void stop() {
        climbMotor.set(0);
    }
}
