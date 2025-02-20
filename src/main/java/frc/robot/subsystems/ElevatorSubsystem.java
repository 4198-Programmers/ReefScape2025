package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  // Create a SparkMax object for the elevator motor
  SparkMax elevatorMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);

  private SparkMaxConfig sparkConfig;
  

  
  public DigitalInput limitSwitchTop = new DigitalInput(Constants.ElevatorConstants.LIMIT_SWITCH_ELEVATOR_TOP);
  public double steadyValue;

  public ElevatorSubsystem() {
    sparkConfig = new SparkMaxConfig();
    sparkConfig
      .idleMode(IdleMode.kBrake);
    elevatorMotor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Create an Encoder object for the elevator motor
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  /**
   *  Sets the elevator motor to a given speed
   *  Negative is up and positive is down 
   */
  public void move(double speed) {
    elevatorMotor.set(speed);
  }

  /**
   * Checks the status of the limit switch
   * @return True if the limit switch is pressed (elevator is at the top)
  */
  public boolean checkSwitch() {
    return limitSwitchTop.get();
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
  }

  /**
   * Gets Encoder value for the steady command
   *
   * @return Steady encoder Value
   */
  public double getSteadyEncoderPosition() {
    return steadyValue;
  }
}
