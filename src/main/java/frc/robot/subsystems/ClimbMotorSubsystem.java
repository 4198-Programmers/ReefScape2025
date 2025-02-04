package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbMotorSubsystem extends SubsystemBase{
    
    private SparkMax climbMotor = new SparkMax(Constants.CLIMB_MOTOR_ID, MotorType.kBrushless);

    public void setClimbSpeed(double speed) {
        climbMotor.set(speed);
    }
    
    public void stop() {
        climbMotor.set(0);
    }
}
