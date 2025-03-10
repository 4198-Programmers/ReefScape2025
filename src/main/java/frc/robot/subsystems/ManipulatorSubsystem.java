package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorSubsystem extends SubsystemBase {
    private SparkMax manipulatorMotor = new SparkMax(ManipulatorConstants.PRIMARY_JOINT_MOTOR_ID, MotorType.kBrushless); // Turns the primary joint closest to the elevator

    private RelativeEncoder manipulatorEncoder = manipulatorMotor.getEncoder();
    final double deadband = ManipulatorConstants.MANIPULATOR_MOTOR_DEADBAND;

    private SparkMaxConfig manipulatorConfig;
    private SparkClosedLoopController manipulatorPID;


    public ManipulatorSubsystem() {
        manipulatorEncoder.setPosition(0);

        manipulatorPID = manipulatorMotor.getClosedLoopController();
        manipulatorEncoder.setPosition(0);
        manipulatorConfig = new SparkMaxConfig();
        manipulatorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.02,0.0,0.03)
            .outputRange(-1, 1);

            manipulatorMotor.configure(manipulatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    /**
     * Turns the primary joint of the manipulator.
     * @param speed The speed at which to turn the joint. Positive values turn it up, negative values turn it down.
     */
    public void turnPrimaryJoint(double speed) { // More Manual Turning
        manipulatorMotor.set(speed);
    }

    /**
     * Turns the primary joint of the manipulator to a given position.
     */
    public void turnPrimaryJointToPosition(double position) {
        manipulatorPID.setReference(position, ControlType.kPosition);
    }

    public Command TurnToPointCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                turnPrimaryJointToPosition(70);
            }
        );
    }

    /**
     * Inline command to turn the primary joint of the manipulator.
     * I did it this way so we could use the runOnce feature to toggle it
     */
    public void zeroManipulator() {
        manipulatorEncoder.setPosition(0);
    }

    public void getEncoder() {
        // System.out.println(manipulatorEncoder.getPosition());
    }

    public Command ZeroManipulatorCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                zeroManipulator();
            }
        );
    }

    public Command GetManipulatorEncoder() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                getEncoder();
            }
        );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //  System.out.println(manipulatorEncoder.getPosition());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}