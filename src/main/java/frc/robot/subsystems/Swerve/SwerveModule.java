package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.CANBus;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class SwerveModule {

    private SparkMax driveMotor;
    private SparkMax angleMotor;
    private RelativeEncoder driveEncoder;
    private CANcoder angleEncoder;
    private CANcoderConfiguration angleEncoderConfiguration;
    private PIDController angleController;
    private SparkMaxConfig driveMotorConfig;
    private double angleAsDouble;
    private double angleEncoderOffset;
    private int moduleNumber;
    private AbsoluteEncoder motorAbsoluteEncoder;
    private SparkClosedLoopController anglePID;
    private SparkMaxConfig turningConfig;
    private SparkMaxConfig driveConfig;
    private RelativeEncoder turningRelativeEncoder;
    

    //Constructor that allows for all of the modules to be created in the subsytem by feeding in the ids and offsets
    public SwerveModule(int driveMotorID, int angleMotorID, int CANCoderID, double angleEncoderOffset, int moduleNumber, boolean invertDriveMotor){

        //Defines the motor in the constructor to tell the rest of the clas it exists
        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);

        driveConfig = new SparkMaxConfig();
        driveConfig
            .idleMode(IdleMode.kBrake)
            .inverted(invertDriveMotor);
        driveConfig.encoder
            .velocityConversionFactor(Constants.DRIVE_VELOCITY_CONVERSION_FACOTR)
            .positionConversionFactor(Constants.DRIVE_POSITION_CONVERSION_FACTOR);

        //Defines the drive encoder of off the drive motor to tell us how many times the robot has spun
        driveEncoder = driveMotor.getEncoder();


        //Defines the motor and the CANCoder
        angleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);
        angleEncoder = new CANcoder(CANCoderID);
        turningRelativeEncoder = angleMotor.getEncoder();
        motorAbsoluteEncoder = angleMotor.getAbsoluteEncoder();
        anglePID = angleMotor.getClosedLoopController();
        turningConfig = new SparkMaxConfig();
        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.009, 0.0000007, 0.0000) //0.15 0.0 2.6, 0.02 0.00003 0
            .outputRange(-1, 1)
            .positionWrappingInputRange(-Math.PI, Math.PI)
            .positionWrappingEnabled(true);
        // turningConfig.encoder
            // .positionConversionFactor(Constants.ANGLE_CONVERSION_FACTOR);
        turningConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20);
        angleMotor.configure(turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        //Makes a configurator object for the CANCoder allowing us to change specific parts of it
        angleEncoderConfiguration = new CANcoderConfiguration();

        //Changes the magnet sensor settings to work better with our robot
        angleEncoderConfiguration.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Constants.ABSOLUTE_ENCODER_DISCONTINUITY_POINT);
        angleEncoderConfiguration.MagnetSensor.MagnetOffset = angleEncoderOffset;
        angleEncoderConfiguration.MagnetSensor.withSensorDirection(Constants.ABSOLUTE_ENCODER_SENSOR_DIRECTION);

        //applies the changes that were made to the CANCoder
        angleEncoder.getConfigurator().apply(angleEncoderConfiguration);    
        
        // angleController = new PIDController(0.05, 0.004, 0.0000);
        // angleController.enableContinuousInput(-Math.PI, Math.PI);
        resetToAbsolute();
        this.moduleNumber = moduleNumber;    
    }

    public void resetToAbsolute() {
        double absolutePosition = angleEncoder.getAbsolutePosition().getValueAsDouble();
        turningRelativeEncoder.setPosition(absolutePosition); 
    }

    //Returns the module angle in degrees;
    public Rotation2d getAngle(){
        //Gets the CTRE value from -0.5 to 0.5

        double angleAsDouble = angleEncoder.getAbsolutePosition().getValueAsDouble();
        // turningRelativeEncoder.setPosition(angleAsDouble);
        // double findingOut = turningRelativeEncoder.getPosition();

        // double angleAsDouble = motorAbsoluteEncoder.getPosition();
        //Multiplies the value of -0.5 to 0.5 giving us the value as an angle

        // System.out.println(findingOut);

        double moduleAngle = (360 * angleAsDouble);
        // System.err.println(moduleAngle);
        double moduleAngleRadians = moduleAngle * (Math.PI / 180);
        turningRelativeEncoder.setPosition(moduleAngleRadians);
        // System.out.println(moduleAngleRadians);
        // System.out.println(turningRelativeEncoder.getPosition());

        //angleMotor.set(0.1);


        // System.out.println("Module Number: " + moduleNumber + "Current Angle: " + new Rotation2d(moduleAngleRadians));

        return new Rotation2d(moduleAngleRadians);

        
    }

    public double getAngleOffset(){
        double angleAsDouble = angleEncoder.getAbsolutePosition().getValueAsDouble();
        return angleAsDouble;
    }

    //Gets the state of this module by giving you the wheel veloicty and the value of the wheel angle
    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    //Gets the current position of the robot or where it is
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    //Sets the desired wheel state of this module for the robot   
    public void setDesiredState(SwerveModuleState desiredStates){
        //used to prevent the robot wheels from spinning further that 90 degrees
        Rotation2d moduleAngle = getAngle();
        // System.out.println("Angle: " + moduleAngle + " Module Number: " + moduleNumber);
        // desiredStates.optimize(moduleAngle);
        
        anglePID.setReference(desiredStates.angle.getRadians(), ControlType.kPosition);
        // double angleOutput = angleController.calculate(getState().angle.getRadians(), desiredStates.angle.getRadians());
        // angleMotor.set(angleOutput);
        driveMotor.set(desiredStates.speedMetersPerSecond);
        System.out.println("Module Number: " + moduleNumber + " Desired Angle: " + desiredStates.angle.getDegrees() + "Current Angle: " + (turningRelativeEncoder.getPosition() * (180/Math.PI)));

    }

    public int getNumber(){
        return moduleNumber;
    }
    
}
