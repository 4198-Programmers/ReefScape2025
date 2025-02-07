package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorSubsystem extends SubsystemBase {
    private SparkMax intakeMotor = new SparkMax(ManipulatorConstants.INTAKE_MOTOR_ID, MotorType.kBrushless); // Motor for the wheels to suck in algae and coral
    private SparkMax rotatingMotor = new SparkMax(ManipulatorConstants.ROTATING_MOTOR_ID, MotorType.kBrushless); // Rotates the end of the manipulator
    private SparkMax primaryJointMotor = new SparkMax(ManipulatorConstants.PRIMARY_JOINT_MOTOR_ID, MotorType.kBrushless); // Turns the primary joint closest to the elevator

    private RelativeEncoder primaryJointEncoder = primaryJointMotor.getEncoder();

    final double deadband = ManipulatorConstants.MANIPULATOR_MOTOR_DEADBAND;
    private boolean isRotated = false;

    public ManipulatorSubsystem() {
        
    } //Maybe invert one of the motors if they're set up flipped so they can be the same direction 


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
    }

    public void runIntake(double speed) { //Both to intake and outtake
        intakeMotor.set(speed);
    }

    public void toggleRotateIntake() { // Toggles the end between 0 and 90 degrees
        double currentPosition = rotatingMotor.getEncoder().getPosition(); // Gets the current position of the rotating motor
        double targetPosition = isRotated ? 0.0 : 90.0; // Target position is 90 degrees if not rotated, otherwise 0 degrees

        if (currentPosition < targetPosition - deadband) { // If the current position is less than the target position it moves it forward
                rotatingMotor.set(0.5);
        } else if (currentPosition > targetPosition + deadband) { // If the current position is greater than the target position it moves it back
                rotatingMotor.set(-0.5);
        } else { // If the current position is equal to the target position
                rotatingMotor.set(0);
                isRotated = !isRotated; // Toggle the state
        }
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