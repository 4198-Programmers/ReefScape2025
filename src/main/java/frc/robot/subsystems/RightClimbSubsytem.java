package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RightClimbSubsytem extends SubsystemBase{
    private SparkMax rightClimbMotor = new SparkMax(Constants.RIGHT.CLIMB.MOTOR.ID, MotorType.kBrushless);

    
}
