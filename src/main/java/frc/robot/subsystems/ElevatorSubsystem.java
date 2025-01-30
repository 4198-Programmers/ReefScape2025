package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  // Create a SparkMax object for the elevator motor
  SparkMax elevatorMotor = new SparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);

  // Create an Encoder object for the elevator motor
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
}
    
  // Sets the elevator motor to a given speed
  public void move(double speed) {
    elevatorMotor.set(speed);
  }

  // Checks if the elevator is at the top
  if (limitSwitchTop.get()) {
    elevatorMotor.set(0);
  }

