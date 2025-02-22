package frc.robot.subsystems.Swerve;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase{

    //Defines the gyro
    public AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private SwerveDriveOdometry odometry;

    private SwerveModule frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule, backRightSwerveModule;

    private final SwerveModule[] modules;

    public SwerveSubsystem(){
        //Added the parameters to define all of the 4 modules
        frontLeftSwerveModule = new SwerveModule(
            Constants.FRONT_LEFT_DRIVE_MOTOR_ID, 
            Constants.FRONT_LEFT_ANGLE_MOTOR_ID, 
            Constants.FRONT_LEFT_CANCODER_ID, 
            Constants.FRONT_LEFT_ANGLE_OFFSET,
            Constants.FRONT_LEFT_MODULE_NUMBER,
            Constants.FRONT_LEFT_DRIVE_INVERT);
        frontRightSwerveModule = new SwerveModule(
            Constants.FRONT_RIGHT_DRIVE_MOTOR_ID, 
            Constants.FRONT_RIGHT_ANGLE_MOTOR_ID, 
            Constants.FRONT_RIGHT_CANCODER_ID, 
            Constants.FRONT_RIGHT_ANGLE_OFFSET,
            Constants.FRONT_RIGHT_MODULE_NUMBER,
            Constants.FRONT_RIGHT_DRIVE_INVERT);
        backLeftSwerveModule = new SwerveModule(
            Constants.BACK_LEFT_DRIVE_MOTOR_ID, 
            Constants.BACK_LEFT_ANGLE_MOTOR_ID, 
            Constants.BACK_LEFT_CANCODER_ID, 
            Constants.BACK_LEFT_ANGLE_OFFSET,
            Constants.BACK_LEFT_MODULE_NUMBER,
            Constants.BACK_LEFT_DRIVE_INVERT);
        backRightSwerveModule = new SwerveModule(
            Constants.BACK_RIGHT_DRIVE_MOTOR_ID, 
            Constants.BACK_RIGHT_ANGLE_MOTOR_ID, 
            Constants.BACK_RIGHT_CANCODER_ID, 
            Constants.BACK_RIGHT_ANGLE_OFFSET,
            Constants.BACK_RIGHT_MODULE_NUMBER,
            Constants.BACK_RIGHT_DRIVE_INVERT);

        odometry = new SwerveDriveOdometry(Constants.SWERVE_DRIVE_KINEMATICS, gyro.getRotation2d(), getSwerveModulePositions());
        modules = new SwerveModule[]{frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule, backRightSwerveModule};
    }   

    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), getSwerveModulePositions());
    }
    //Returns all the swerve module states
    public SwerveModuleState[] getSwerveModuleStates(){
        return new SwerveModuleState[]{
            frontLeftSwerveModule.getState(),
            frontRightSwerveModule.getState(),
            backLeftSwerveModule.getState(),
            backRightSwerveModule.getState()
        };
    }

    public void getModuleAngles(){
        System.out.println("Front Left Module Angle: " + frontLeftSwerveModule.getAngleOffset() +
         " Front Right Module Angle: " + frontRightSwerveModule.getAngleOffset()         + 
         " Back Left Module Angle: " + backLeftSwerveModule.getAngleOffset() + 
         " Back Right Module Angle: " + backRightSwerveModule.getAngleOffset());
    }



    //Returns the positions of all swerve modules
    public SwerveModulePosition[] getSwerveModulePositions(){
        return new SwerveModulePosition[]{
            frontLeftSwerveModule.getPosition(),
            frontRightSwerveModule.getPosition(),
            backLeftSwerveModule.getPosition(),
            backRightSwerveModule.getPosition()
        };
    }

    public void setSwerveModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_DRIVE_SPEED_MPS);
        for(SwerveModule module : modules){
            if(module.getAngle().getDegrees() > desiredStates[module.getNumber()].angle.getDegrees() + 1 || module.getAngle().getDegrees() < desiredStates[module.getNumber()].angle.getDegrees() - 1 ) {
                module.setDesiredState(new SwerveModuleState(module.getState().speedMetersPerSecond, module.getAngle()));
            } else {
            module.setDesiredState(desiredStates[module.getNumber()]);
            }
        }
    }

    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented){
        SwerveModuleState[] states;
        if (fieldOriented){
        states = Constants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, gyro.getRotation2d()));
        }
        else {
        states = Constants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
        }
        setSwerveModuleStates(states);
    }
}
