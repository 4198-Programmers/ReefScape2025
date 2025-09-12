package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {

    private SwerveModule frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule, backRightSwerveModule;
    private SwerveDriveOdometry odometry;
    public AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    
    public SwerveSubsystem() {
        frontLeftSwerveModule = new SwerveModule(
                Constants.FRONT_LEFT_DRIVE_MOTOR_ID, 
                Constants.FRONT_LEFT_ANGLE_MOTOR_ID,
                Constants.FRONT_LEFT_ANGLE_ENCODER_ID, 
                Constants.FRONT_LEFT_ANGLE_OFFSET,
                1);
        frontRightSwerveModule = new SwerveModule(
                Constants.FRONT_RIGHT_DRIVE_MOTOR_ID, 
                Constants.FRONT_RIGHT_ANGLE_MOTOR_ID, 
                Constants.FRONT_RIGHT_ANGLE_ENCODER_ID, 
                Constants.FRONT_RIGHT_ANGLE_OFFSET,
                2);
        backLeftSwerveModule = new SwerveModule(
                Constants.BACK_LEFT_DRIVE_MOTOR_ID, 
                Constants.BACK_LEFT_ANGLE_MOTOR_ID, 
                Constants.BACK_LEFT_ANGLE_ENCODER_ID, 
                Constants.BAKC_LEFT_ANGLE_OFFSET,
                3);
        backRightSwerveModule = new SwerveModule(
                Constants.BACK_RIGHT_DRIVE_MOTOR_ID, 
                Constants.BACK_RIGHT_ANGLE_MOTOR_ID, 
                Constants.BACK_RIGHT_ANGLE_ENCODER_ID, 
                Constants.BACK_RIGHT_ANGLE_OFFSET,
                4);

        odometry = new SwerveDriveOdometry(Constants.SWERVE_DRIVE_KINEMATICS, gyro.getRotation2d().times(-1), getSwerveModulePositions());
        resetGyro();

        RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> {
                ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                gyro.getRotation2d() // Replace with your method to get the robot's current heading as a Rotation2d
            );

             // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            driveRobotRelative(robotRelativeSpeeds);
            },
             new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(.1, 0.0, 0.01), // Translation PID constants
                    new PIDConstants(.3, 0.0, 0.01) // Rotation PID constants
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

    

    public void resetGyro() {
        gyro.reset();
    }

    @Override
    public void periodic() {
        // odometry.update(gyro.getRotation2d(), getSwerveModulePositions());
        // System.out.println(getRobotRelativeSpeeds());
        // System.out.println(frontLeftSwerveModule.getState());
        // System.out.println(gyro.getRotation2d());
        // System.out.println(getRobotRelativeSpeeds());
        // System.out.println(getPose().getRotation().getDegrees());

    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }
    
    public void resetPose(Pose2d pose){
        odometry.resetPosition(gyro.getRotation2d().times(-1), getSwerveModulePositions(), pose);
        resetGyro();
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = Constants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        setModuleStates(states);
        // System.out.println("Robot Relative Speeds: " + speeds.vxMetersPerSecond + ", " + speeds.vyMetersPerSecond + ", " + speeds.omegaRadiansPerSecond);
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(frontLeftSwerveModule.getState(), 
                                                                frontRightSwerveModule.getState(), 
                                                                backLeftSwerveModule.getState(), 
                                                                backRightSwerveModule.getState());
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[]{
            frontLeftSwerveModule.getPosition(),
            frontRightSwerveModule.getPosition(),
            backLeftSwerveModule.getPosition(),
            backRightSwerveModule.getPosition()
        };
    }

    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented){
        SwerveModuleState[] states;
        Rotation2d dumDum = Rotation2d.fromRotations(zSpeed);
        if(fieldOriented) {
            states = Constants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed * Constants.MAX_VELOCITY_MPS, ySpeed * Constants.MAX_VELOCITY_MPS, -dumDum.getRadians(), gyro.getRotation2d()));
            // System.out.println("Radians: " + dumDum.getRadians());
            
        } else {
            states = Constants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(xSpeed * Constants.MAX_VELOCITY_MPS, ySpeed * Constants.MAX_VELOCITY_MPS, zSpeed / 5));
        } 
        setModuleStates(states);
    }

    public Command getAutonomousCommand(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_VELOCITY_MPS);
        frontLeftSwerveModule.setDesiredState(states[0]);
        frontRightSwerveModule.setDesiredState(states[1]);
        backLeftSwerveModule.setDesiredState(states[2]);
        backRightSwerveModule.setDesiredState(states[3]);

    }

}
