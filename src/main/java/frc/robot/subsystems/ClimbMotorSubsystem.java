package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbMotorSubsystem extends SubsystemBase{
    
    private SparkMax climbMotor;
    private RelativeEncoder climbEncoder;
    private SparkMaxConfig climbConfigurator;
    private SparkClosedLoopController climbPID;

    public ClimbMotorSubsystem() {
        climbMotor = new SparkMax(Constants.ClimbConstants.CLIMB_MOTOR_ID, MotorType.kBrushless);
        climbEncoder = climbMotor.getEncoder();
        climbConfigurator = new SparkMaxConfig();
        climbPID = climbMotor.getClosedLoopController();

        climbConfigurator.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1,0,0)
            .outputRange(-1, 1);

        climbMotor.configure(climbConfigurator, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

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
