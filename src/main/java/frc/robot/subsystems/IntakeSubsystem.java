package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.security.BasicPermission;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    private SparkMax intakeMotor = new SparkMax(ManipulatorConstants.INTAKE_MOTOR_ID, MotorType.kBrushless); // Motor for the wheels to suck in algae and coral
    private SparkMax intakeMotorTwo = new SparkMax(ManipulatorConstants.INTAKE_MOTOR_TWO_ID, MotorType.kBrushless); // Motor for the wheels to suck in algae and coral

    /**
     * Runs the intake motor at a given speed.
     * @param speed The speed at which to run the intake motor. Positive values for outtake, negative values for intake.
     */
    public void runIntake(double speed) { //Both to intake and outtake
        intakeMotor.set(speed);
        // intakeMotorTwo.set(speed);
    }

    public void runOutake(double speed) {
        intakeMotor.set(speed);
        intakeMotorTwo.set(speed);
    }
}
