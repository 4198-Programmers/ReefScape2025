package frc.robot.subsystems.Swerve;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class SwerveSubsystem extends SubsystemBase{

    //Defines the gyro
    public AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private SwerveDriveOdometry odometry;

    private SwerveModule frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule, backRightSwerveModule;
    private List<String> recordedInputs = new ArrayList<>();

    private final SwerveModule[] modules;

    public SwerveSubsystem(){
        //Added the parameters to define all of the 4 modules
        gyro.reset();
        frontLeftSwerveModule = new SwerveModule(
            Constants.FRONT_LEFT_DRIVE_MOTOR_ID, 
            Constants.FRONT_LEFT_ANGLE_MOTOR_ID, 
            Constants.FRONT_LEFT_CANCODER_ID, 
            Constants.FRONT_LEFT_ANGLE_OFFSET,
            Constants.FRONT_LEFT_MODULE_NUMBER,
            Constants.FRONT_LEFT_DRIVE_INVERT,
            Constants.FRONT_LEFT_ANGLE_INVERT);
        frontRightSwerveModule = new SwerveModule(
            Constants.FRONT_RIGHT_DRIVE_MOTOR_ID, 
            Constants.FRONT_RIGHT_ANGLE_MOTOR_ID, 
            Constants.FRONT_RIGHT_CANCODER_ID, 
            Constants.FRONT_RIGHT_ANGLE_OFFSET,
            Constants.FRONT_RIGHT_MODULE_NUMBER,
            Constants.FRONT_RIGHT_DRIVE_INVERT,
            Constants.FRONT_RIGHT_ANGLE_INVERT);
        backLeftSwerveModule = new SwerveModule(
            Constants.BACK_LEFT_DRIVE_MOTOR_ID, 
            Constants.BACK_LEFT_ANGLE_MOTOR_ID, 
            Constants.BACK_LEFT_CANCODER_ID, 
            Constants.BACK_LEFT_ANGLE_OFFSET,
            Constants.BACK_LEFT_MODULE_NUMBER,
            Constants.BACK_LEFT_DRIVE_INVERT,
            Constants.BACK_LEFT_ANGLE_INVERT);
        backRightSwerveModule = new SwerveModule(
            Constants.BACK_RIGHT_DRIVE_MOTOR_ID, 
            Constants.BACK_RIGHT_ANGLE_MOTOR_ID, 
            Constants.BACK_RIGHT_CANCODER_ID, 
            Constants.BACK_RIGHT_ANGLE_OFFSET,
            Constants.BACK_RIGHT_MODULE_NUMBER,
            Constants.BACK_RIGHT_DRIVE_INVERT,
            Constants.BACK_RIGHT_ANGLE_INVERT);

        odometry = new SwerveDriveOdometry(Constants.SWERVE_DRIVE_KINEMATICS, gyro.getRotation2d().times(-1), getSwerveModulePositions());
        modules = new SwerveModule[]{frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule, backRightSwerveModule};

        RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      System.out.println(e);
    }


        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.001, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.001, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );


    }   

    public void setGyroAngle(double angle){
        gyro.setAngleAdjustment(angle);
    }


    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d().times(-1), getSwerveModulePositions());
        // System.out.println(gyro.getRotation2d());
        // System.out.println(getPose());
        // Debugging: Print the current pose and module states
       // System.out.println("Current Pose: " + getPose());
      //  for (SwerveModule module : modules) {
            //System.out.println("Module " + module.getNumber() + " State: " + module.getState());
      //  }
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(frontLeftSwerveModule.getState(), 
                                                                frontRightSwerveModule.getState(), 
                                                                backLeftSwerveModule.getState(), 
                                                                backRightSwerveModule.getState());
    }

    public void resetPose(Pose2d pose){
        odometry.resetPosition(gyro.getRotation2d().times(-1), getSwerveModulePositions(), pose);
    }

    public void resetGyro(){
        gyro.reset();
        // resetPose(new Pose2d(0, 0, new Rotation2d(Math.PI)));
    }



    public void resetOdometryPose() {
        resetPose(new Pose2d(0, 0, new Rotation2d(Math.PI)));
    }

    /**
     * Returns the current pose of the robot
     * @return
     */
    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public void recordInput(double xSpeed, double ySpeed, double zSpeed){
        System.out.println("Recording");
        recordedInputs.add(String.format("%.2f,%.2f,%.2f,%b,%b,%d", xSpeed, ySpeed, zSpeed));
    }

    public void logRecordedInputs(){
        // Save recorded data to a file when the robot is disabled
        try (FileWriter writer = new FileWriter("../joystick_recording.csv")) {
            for (String row : recordedInputs) {
                writer.write(row + "\n");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public Command logInputsCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                logRecordedInputs();
            }
        );
    }

    
    /**
     * Returns all the swerve module states
     * @return SwerveModuleState[] of all swerve modules
     */
    public SwerveModuleState[] getSwerveModuleStates(){
        return new SwerveModuleState[]{
            frontLeftSwerveModule.getState(),
            frontRightSwerveModule.getState(),
            backLeftSwerveModule.getState(),
            backRightSwerveModule.getState()
        };
    }

    public void resetToAbsolutes(){
        frontLeftSwerveModule.resetToAbsolute();
        frontRightSwerveModule.resetToAbsolute();
        backLeftSwerveModule.resetToAbsolute();
        backRightSwerveModule.resetToAbsolute();
    }

    public void getModuleAngles(){
        // System.out.println("Front Left Module Angle: " + frontLeftSwerveModule.getAngleOffset() +
        //  " Front Right Module Angle: " + frontRightSwerveModule.getAngleOffset()         + 
        //  " Back Left Module Angle: " + backLeftSwerveModule.getAngleOffset() + 
        //  " Back Right Module Angle: " + backRightSwerveModule.getAngle().getDegrees());
    }



    /**
     * Returns the positions of all swerve modules
     * @return SwerveModulePosition[] of all swerve modules
     */
    public SwerveModulePosition[] getSwerveModulePositions(){
        return new SwerveModulePosition[]{
            frontLeftSwerveModule.getPosition(),
            frontRightSwerveModule.getPosition(),
            backLeftSwerveModule.getPosition(),
            backRightSwerveModule.getPosition()
        };
    }
    public double deadband = 0;
    public void setSwerveModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_DRIVE_SPEED_MPS);
        for(SwerveModule module : modules){
            /* if 180 - abs desired + 180 - abs current < deadband
             * 
             */



            // if (Math.abs(desiredStates[module.getNumber()].angle.getDegrees() - module.getAngle().getDegrees()) < deadband || (180 - Math.abs(desiredStates[module.getNumber()].angle.getDegrees())) + (180 - Math.abs(module.getAngle().getDegrees())) < deadband) {
            //     module.setWithinDeadzone(true);
            // } else {
            //     module.setWithinDeadzone(false);
            // }
            module.setDesiredState(desiredStates[module.getNumber()]);

            // if(module.getAngle().getDegrees() > desiredStates[module.getNumber()].angle.getDegrees() + deadband || module.getAngle().getDegrees() > desiredStates[module.getNumber()].angle.getDegrees() + deadband + 180  && module.getAngle().getDegrees() < desiredStates[module.getNumber()].angle.getDegrees() - deadband || module.getAngle().getDegrees() < desiredStates[module.getNumber()].angle.getDegrees() - deadband -180 ) {
            //     System.out.println("Outside of deadzone!!!");
                // System.out.println(desiredStates[module.getNumber()].angle.getDegrees());
            // } else {
            //     System.out.println("Inside Deadzone!!!");
            //     module.setDesiredState(new SwerveModuleState(desiredStates[module.getNumber()].speedMetersPerSecond, module.getAngle()));
                

            // }
        }
    }

    public Command getAutonomousCommand(String pathName) {
        // System.out.println("Hey Babe");
        // System.out.println(pathName);
        return new PathPlannerAuto(pathName);
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
        SwerveModuleState[] states;
        states = Constants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond / 4.69, chassisSpeeds.vyMetersPerSecond / 4.69, chassisSpeeds.omegaRadiansPerSecond));
        // Debugging: Print the desired chassis speeds
       System.out.println("Desired Chassis Speeds: " + chassisSpeeds);
       System.err.println(getRobotRelativeSpeeds());
    // System.err.println("Jackson is AWESOME");
        setSwerveModuleStates(states);
    }

    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented){
        SwerveModuleState[] states;
        if (fieldOriented){
        states = Constants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed, -xSpeed, zSpeed * 0.05, gyro.getRotation2d()));
        }
        else {
        states = Constants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(ySpeed, xSpeed, zSpeed * 0.1));
        }
        // Debugging: Print the desired states
        // for (SwerveModuleState state : states) {
        //     System.out.println("Desired State: " + state);
        // }
        // System.out.println(states);
        setSwerveModuleStates(states);
    }
}
