package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  // Create a SparkMax object for the elevator motor
  SparkMax elevatorMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  // Create an Encoder object for the elevator motor
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  private SparkMaxConfig elevatorConfig;
  private SparkClosedLoopController elevatorPID;
  
  public DigitalInput limitSwitchTop = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_ELEVATOR_TOP);

  public ElevatorSubsystem() {
    elevatorPID = elevatorMotor.getClosedLoopController();
    elevatorEncoder.setPosition(0);
    elevatorConfig = new SparkMaxConfig();
    elevatorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(0.08,0.0,0.02)
          .outputRange(-1.0, 0.7);

    elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   *  Sets the elevator motor to a given speed
   *  Negative is up and positive is down 
   */
  public void moveToPosition(double position) {
    elevatorPID.setReference(position, ControlType.kPosition);
    // System.out.println("Moving to position: " + position);
  }

  /**
   * Checks the status of the limit switch
   * @return True if the limit switch is pressed (elevator is at the top)
  */
  public boolean checkSwitch() {
    System.out.println("Used limit switch! " + limitSwitchTop.get());
    return !limitSwitchTop.get();
  }

  /**
   * Checks the encoder value of the elevator motor
   * @return Encoder value
   */
  public double checkEncoder() {
    return elevatorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // System.out.println(elevatorEncoder.getPosition());
    //System.out.println("Current: " + elevatorMotor.getOutputCurrent());
  }
}
