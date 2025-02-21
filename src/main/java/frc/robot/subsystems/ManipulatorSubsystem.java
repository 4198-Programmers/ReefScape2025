package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorSubsystem extends SubsystemBase {
    private SparkMax primaryJointMotor = new SparkMax(ManipulatorConstants.PRIMARY_JOINT_MOTOR_ID, MotorType.kBrushless); // Turns the primary joint closest to the elevator

    private RelativeEncoder primaryJointEncoder = primaryJointMotor.getEncoder();
    final double deadband = ManipulatorConstants.MANIPULATOR_MOTOR_DEADBAND;


    public ManipulatorSubsystem() {
        
    }


    public void turnPrimaryJoint(double speed) { // More Manual Turning
        primaryJointMotor.set(speed);
    }

    public void turnPrimaryJointExact(double targetPosition) { // Turns the primary joint to a specific position
        double currentPosition = primaryJointEncoder.getPosition(); // Gets the current position of the primary joint

        while (currentPosition < targetPosition-deadband || currentPosition > targetPosition+deadband) { // runs while the current position is not within the deadband of the target position
            if (currentPosition < targetPosition-deadband) { // If the current position is less than the target position it moves it forward
                primaryJointMotor.set(0.5);
            } else if (currentPosition > targetPosition+deadband) { // If the current position is greater than the target position it moves it back
                primaryJointMotor.set(-0.5);
            } else { // If the current position is equal to the target position
                primaryJointMotor.set(0);
                break; // breaks out of the loop if it reached that point
            }
        }
        primaryJointMotor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}