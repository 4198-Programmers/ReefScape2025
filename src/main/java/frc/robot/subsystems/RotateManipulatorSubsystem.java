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

public class RotateManipulatorSubsystem extends SubsystemBase {

    private SparkMax rotatingMotor = new SparkMax(ManipulatorConstants.ROTATING_MOTOR_ID, MotorType.kBrushless); // Rotates the end of the manipulator
    private RelativeEncoder rotatingEncoder = rotatingMotor.getEncoder(); // Encoder for the rotating motor
    private SparkClosedLoopController rotatingPID;
    private SparkMaxConfig rotatingConfig;
    public boolean isRotated = false;
    final double deadband = ManipulatorConstants.MANIPULATOR_MOTOR_DEADBAND;
    double zero = 0;
    boolean hasBeenZeroed = false;
    boolean hasBeenMotorZeroed = false;

    public DigitalInput rotateSensor = new DigitalInput(Constants.ManipulatorConstants.INTAKE_SENSOR_ID);

    public Command RotateManipulatorCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
            double targetPosition = isRotated ? zero : 5.5; // Target position is 90 degrees if not rotated, otherwise 0 degrees
            rotatingPID.setReference(targetPosition, ControlType.kPosition); // Sets the target position of the rotating motor
            System.out.println("Set reference to: " + targetPosition);
            System.out.println("Actual position: " + rotatingEncoder.getPosition());
            isRotated = !isRotated; // Toggles the rotation state
        });
    }

    public RotateManipulatorSubsystem() {
        rotatingPID = rotatingMotor.getClosedLoopController();
        rotatingEncoder.setPosition(0);
        rotatingConfig = new SparkMaxConfig();
        rotatingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.075, 0.0, 1.0)
            .outputRange(-1, 1);
        rotatingMotor.configure(rotatingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


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
        double targetPosition = isRotated ? zero : 5.5; // Target position is 90 degrees if not rotated, otherwise 0 degrees
        rotatingPID.setReference(targetPosition, ControlType.kPosition); // Sets the target position of the rotating motor
        System.out.println("Set reference to: " + targetPosition);
        System.out.println("Actual position: " + rotatingEncoder.getPosition());
        isRotated = !isRotated; // Toggles the rotation state
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
