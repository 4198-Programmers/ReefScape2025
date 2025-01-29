package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private SparkMax intakeMotor = new SparkMax(ManipulatorConstants.INTAKE_MOTOR_ID, MotorType.kBrushless); // Motor for the wheels to suck in algae and coral
  private SparkMax primaryJointMotor = new SparkMax(ManipulatorConstants.PRIMARY_JOINT_MOTOR_ID, MotorType.kBrushless); // Turns the primary joint closest to the elevator
  private SparkMax secondaryJointMotor = new SparkMax(ManipulatorConstants.SECONDARY_JOINT_MOTOR_ID, MotorType.kBrushless); // Turns the central joint

  private RelativeEncoder primaryJointEncoder = primaryJointMotor.getEncoder();
  private RelativeEncoder secondaryJointEncoder = secondaryJointMotor.getEncoder();

  final double deadband = ManipulatorConstants.MANIPULATOR_MOTOR_DEADBAND;

  public ManipulatorSubsystem() {
    
  } //Maybe invert one of the motors if they're set up flipped so they can be the same direction 


  public void turnPrimaryJoint(float speed) { // More Manual Turning
    primaryJointMotor.set(speed);
  }

  public void turnSecondaryJoint(float speed) { // More Manual Turning
    secondaryJointMotor.set(speed);
  }

  public void turnPrimaryJointExact(float targetPosition) { // Turns the primary joint to a specific position
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

  public void turnSecondaryJointExact(float targetPosition) { // Turns the primary joint to a specific position
    double currentPosition = secondaryJointEncoder.getPosition(); // Gets the current position of the primary joint

    while (currentPosition < targetPosition-deadband || currentPosition > targetPosition+deadband) { // runs while the current position is not within the deadband of the target position
      if (currentPosition < targetPosition-deadband) { // If the current position is less than the target position it moves it forward
        secondaryJointMotor.set(0.5);
      } else if (currentPosition > targetPosition+deadband) { // If the current position is greater than the target position it moves it back
        secondaryJointMotor.set(-0.5);
      } else { // If the current position is equal to the target position
        secondaryJointMotor.set(0);
        break; // breaks out of the loop if it reached that point
      }
    }
    
  }

  public void runIntake(float speed) { //Both to intake and outtake
    intakeMotor.set(speed);
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