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

  // Sets the elevator motor to a given speed
  public void move(double speed) {
    elevatorMotor.set(speed);
  }

  /// Checks limit switch
  public boolean checkSwitch() {
    return !limitSwitchTop.get();
  }

  /// Checks Encoder Value
  public double checkEncoder() {
    return elevatorEncoder.getPosition();
  }

  /// Sets encoder value for steady command
  public void setSteadyEncoderPosition() {
    steadyValue = elevatorEncoder.getPosition();
  }

  /**
   * Gets Encoder value for the steady command
   *
   * @return Encoder Value
   */
  public double getSteadyEncoderPosition() {
    return steadyValue;
  }
}
