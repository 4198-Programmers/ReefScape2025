package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorConstants;

/*
 * This subsystem is responsible for rotating the manipulator. Split off from the normal manipulator subsystem so we can run them in tandem.
 */
public class RotateManipulatorSubsystem extends SubsystemBase {

    private SparkMax rotatingMotor = new SparkMax(ManipulatorConstants.ROTATING_MOTOR_ID, MotorType.kBrushless); // Rotates the end of the manipulator
    private RelativeEncoder rotatingEncoder = rotatingMotor.getEncoder(); // Encoder for the rotating motor
    private SparkClosedLoopController rotatingPID;
    private SparkMaxConfig rotatingConfig;
    public boolean isRotated = false;
    double zero = -0.5;
    boolean hasBeenZeroed = false;

    // public DigitalInput rotateSensor = new DigitalInput(Constants.ManipulatorConstants.INTAKE_SENSOR_ID);

    public RotateManipulatorSubsystem() {
        rotatingPID = rotatingMotor.getClosedLoopController();
        rotatingEncoder.setPosition(0);
        rotatingConfig = new SparkMaxConfig();
        rotatingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.11, 0.0, 1.5)
            .outputRange(-0.4, 0.4);
        rotatingMotor.configure(rotatingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Inline command to rotate the manipulator.
     * I did it this way so we could use the runOnce feature to toggle it
     */
    public Command RotateManipulatorCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                double targetPosition = isRotated ? 0 : 8; // Target position is 90 degrees if not rotated, otherwise 0 degrees
                rotatingPID.setReference(targetPosition, ControlType.kPosition); // Sets the target position of the rotating motor
                // System.out.println("Set reference to: " + targetPosition);
                // System.out.println("Actual position: " + rotatingEncoder.getPosition());
                isRotated = !isRotated; // Toggles the rotation state
            }
        );
    }

    /**
     * Rotates the intake to the toggled target position.
     */
    public void toggleRotateIntake() { // Toggles the end between 0 and 90 degrees
        double targetPosition = isRotated ? 0 : 8; // Target position is 90 degrees if not rotated, otherwise 0 degrees
        rotatingPID.setReference(targetPosition, ControlType.kPosition); // Sets the target position of the rotating motor
        isRotated = !isRotated; // Toggles the rotation state
    }

    public void RotateManipulator(double speed){
        rotatingMotor.set(speed);
    }

    /**
     * Sets the intake to a given position.
     * @param position The position to set the intake to.
     */
    public void setIntakePosition(double position) {
        rotatingPID.setReference(position, ControlType.kPosition);
    }

    public void zeroMotor() {
        rotatingMotor.set(0.1);
        try {
            Thread.sleep(300); // Wait for 0.5 seconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        hasBeenZeroed = false;
        rotatingMotor.set(-0.1);
        try {
            Thread.sleep(600); // Wait for 1 second
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        rotatingPID.setReference(0, ControlType.kPosition); // Stop the motor
    }

    public Command ZeroManipulatorRotate() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {

                // System.out.println("Zeroing manipulator");
                zeroMotor();
            }
        );
    }

    // public void periodic() {
    //     if (!rotateSensor.get() && !hasBeenZeroed) {
    //         System.out.println("Zeroing manipulator");
    //         // rotatingEncoder.setPosition(0);
    //         hasBeenZeroed = true;
    //     }
    //     //System.out.println(rotatingEncoder.getPosition());
    // }
}
