package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class RotateManipulatorSubsystem extends SubsystemBase {

    private SparkMax rotatingMotor = new SparkMax(ManipulatorConstants.ROTATING_MOTOR_ID, MotorType.kBrushless); // Rotates the end of the manipulator
    public boolean isRotated = false;
    final double deadband = ManipulatorConstants.MANIPULATOR_MOTOR_DEADBAND;


    /**
     * Toggles the rotation state of the intake.
     */
    public void toggleRotate() {
        isRotated = !isRotated;
    }

    /**
     * Rotates the intake to the toggled target position.
     */
    public void toggleRotateIntake() { // Toggles the end between 0 and 90 degrees
        double currentPosition = rotatingMotor.getEncoder().getPosition() * 360; // Gets the current position of the rotating motor
        double targetPosition = isRotated ? 0.0 : 90.0; // Target position is 90 degrees if not rotated, otherwise 0 degrees

        System.out.println("Encoder: " + currentPosition + ". Target position:" + targetPosition);
        if (currentPosition < targetPosition - deadband) { // If the current position is less than the target position it moves it forward
                rotatingMotor.set(0.04);
        } else if (currentPosition > targetPosition + deadband) { // If the current position is greater than the target position it moves it back
                rotatingMotor.set(-0.04);
        } else { // If the current position is equal to the target position
                rotatingMotor.set(0);
        }
    }

    /**
     * Stops the rotation of the intake.
     */
    public void stopRotate() {
        rotatingMotor.set(0);
    }
}
