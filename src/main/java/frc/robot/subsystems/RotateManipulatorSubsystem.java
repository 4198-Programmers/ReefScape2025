package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorConstants;

public class RotateManipulatorSubsystem extends SubsystemBase {

    private SparkMax rotatingMotor = new SparkMax(ManipulatorConstants.ROTATING_MOTOR_ID, MotorType.kBrushless); // Rotates the end of the manipulator
    public boolean isRotated = false;
    final double deadband = ManipulatorConstants.MANIPULATOR_MOTOR_DEADBAND;
    double zero = -0.3;
    boolean hasBeenZeroed = false;
    boolean hasBeenMotorZeroed = false;

    public DigitalInput rotateSensor = new DigitalInput(Constants.ManipulatorConstants.INTAKE_SENSOR_ID);


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
        double currentPosition = rotatingMotor.getEncoder().getPosition(); // Gets the current position of the rotating motor
        if (rotateSensor.get() && !hasBeenZeroed) {
            System.out.println("Passed the sensor!!");
            zero = currentPosition;
            // rotatingMotor.getEnncoder().
            hasBeenZeroed = true;
        }

        double targetPosition = isRotated ? zero : -6.5; // Target position is 90 degrees if not rotated, otherwise 0 degrees
        
        // if (hasBeenMotorZeroed == false) {
        //     zero = currentPosition;
        //     hasBeenZeroed = true;
        // }

        // System.out.println("Encoder: " + currentPosition + ". Target position:" + targetPosition);
        if (currentPosition < targetPosition - deadband) { // If the current position is less than the target position it moves it forward
                rotatingMotor.set(0.1);
        } else if (currentPosition > targetPosition + deadband) { // If the current position is greater than the target position it moves it back
                rotatingMotor.set(-0.1);
        } else { // If the current position is equal to the target position
                rotatingMotor.set(0);
        }
        // System.out.println(currentPosition);
    }

    public void zeroMotor() {

    }

    /**
     * Stops the rotation of the intake.
     */
    public void stopRotate() {
        rotatingMotor.set(0);
    }
}
