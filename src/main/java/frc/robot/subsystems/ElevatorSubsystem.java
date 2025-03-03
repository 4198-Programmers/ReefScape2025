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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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

  

  
  public DigitalInput limitSwitchTop = new DigitalInput(Constants.ElevatorConstants.LIMIT_SWITCH_ELEVATOR_TOP);
  public double steadyValue;

  public ElevatorSubsystem() {
    elevatorPID = elevatorMotor.getClosedLoopController();
    elevatorEncoder.setPosition(0);
    elevatorConfig = new SparkMaxConfig();
    elevatorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(0.5,0,0.1)
          .outputRange(-1, 1);

    elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  /**
   *  Sets the elevator motor to a given speed
   *  Negative is up and positive is down 
   */
  public void moveToPosition(int position) {
    elevatorPID.setReference(position, ControlType.kPosition);
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

  /**
   * Sets the steady encoder position to the current encoder value
   * used in commands to keep the elevator steady to keep track of where to keep it
   */
  public void setSteadyEncoderPosition() {
    steadyValue = elevatorEncoder.getPosition();
    System.out.println(steadyValue);
  }

  /**
   * Gets Encoder value for the steady command
   *
   * @return Steady encoder Value
   */
  public double getSteadyEncoderPosition() {
    return steadyValue;
  }

  public void elevatorTargetPosition(double speed, double elevatorTargetPosition){
    System.out.println("Elevator Encoder: " + elevatorEncoder.getPosition());
    while(Math.abs(elevatorTargetPosition - elevatorEncoder.getPosition()) > ElevatorConstants.ELEVATOR_DEADBAND){
      if (elevatorEncoder.getPosition() < elevatorTargetPosition-ElevatorConstants.ELEVATOR_DEADBAND){
        elevatorMotor.set(speed); //moves the elevator up if it is below the deadband range
        System.out.println("Elevator Encoder: " + elevatorEncoder.getPosition());

      } else if (elevatorEncoder.getPosition() > (elevatorTargetPosition + ElevatorConstants.ELEVATOR_DEADBAND)){
        elevatorMotor.set(-speed); //moves the elevator down if it is above the deadband range
        System.out.println("Elevator Encoder: " + elevatorEncoder.getPosition());

      } else { 
        elevatorMotor.set(0);
        System.out.println("Elevator Encoder: " + elevatorEncoder.getPosition());

        break; //stops motor and breaks out of loop if the elevator is within the deadband range
      }
    }
  elevatorMotor.set(0);
}
}
