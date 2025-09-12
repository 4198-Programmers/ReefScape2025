// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.geometry.Rotation3d;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


    public static class ElevatorConstants {
        public static final int ELEVATOR_MOTOR_ID = 9;
        public static final int LIMIT_SWITCH_ELEVATOR_TOP = 1;

        public static final int ELEVATOR_BUTTON_POSITION_ONE = 9;
        public static final int ELEVATOR_BUTTON_POSITION_TWO = 10;
        public static final int ELEVATOR_BUTTON_POSITION_THREE = 11;
        public static final int ELEVATOR_BUTTON_POSITION_FOUR = 12;

        // Elevator positions in rotations
        public static final double ELEVATOR_POSITION_0 = -5; // Close enough to zero but enough that it doesn't crash down
        public static final double ELEVATOR_POSITION_1 = -15; // First reef section
        public static final double ELEVATOR_POSITION_2 = -40; // Second reef section
        public static final double ELEVATOR_POSITION_3 = -72.5; // Third reef section

        // public static final int ELEVATOR_UP_BUTTON = 6; // Buttons on right side of buttons joystick
        // public static final int ELEVATOR_DOWN_BUTTON = 4;
    }

    public static class ClimbConstants {
        public static final int CLIMB_MOTOR_ID = 13;
        public static final double CLIMB_SPEED = 1;
        public static final int CLIMB_FORWARD_BUTTON = 8;
        public static final int CLIMB_REVERSE_BUTTON = 7;

    }
    public static final int JOYSTICK_RIGHT_ID = 2;
    public static final int JOYSTICK_MIDDLE_ID = 1;
    public static final int JOYSTICK_LEFT_ID = 0;

  public static class ManipulatorConstants {
    public static final int INTAKE_MOTOR_ID = 12; 
    public static final int PRIMARY_JOINT_MOTOR_ID = 10;
    public static final double MANIPULATOR_MOTOR_DEADBAND = 0.15;
    public static final int ROTATING_MOTOR_ID = 11;
    public static final double MANIPULATOR_MOTOR_SPEED = 1.0;
    public static final double INTAKE_MOTOR_SPEED = 1.0;
    public static final int INTAKE_SENSOR_ID = 0;

    public static final int MANIPULATOR_ROTATE_BUTTON = 5;
    public static final int INTAKE_MOTOR_TWO_ID = 14;

    public static final double INTAKE_ZERO = -0.25;
    public static final double INTAKE_ROTATED = 8;
  }
  
  public static final double WHEEL_DIAMETER = Units.inchesToMeters(4); // 4 inch diameter wheel

public static final double SWERVE_ANGLE_GEAR_RATIO = 1 / 12.8;
public static final double SWERVE_DRIVE_GEAR_RATIO = 1 / 6.75;

// public static final double SWERVE_DRIVE_VELOCITY_CONVERSION_FACTOR = (WHEEL_DIAMETER * Math.PI) / SWERVE_DRIVE_GEAR_RATIO;
public static final double SWERVE_DRIVE_VELOCITY_CONVERSION_FACTOR = SWERVE_DRIVE_GEAR_RATIO / 60;

public static final double DRIVETRAIN_LENGTH = Units.inchesToMeters(29.75);  //Need to be distance between wheels
public static final double DRIVETRAIN_WIDTH = Units.inchesToMeters(29.75);

public static final double MAX_VELOCITY_MPS = Units.feetToMeters(14.75);

public static final double MAX_ROTATIONAL_VELOCITY = MAX_VELOCITY_MPS / Math.hypot(
      SWERVE_ANGLE_GEAR_RATIO / 2, 
      SWERVE_ANGLE_GEAR_RATIO / 2);

//Swerve Front Left
public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
public static final int FRONT_LEFT_ANGLE_MOTOR_ID = 2;
public static final int FRONT_LEFT_ANGLE_ENCODER_ID = 13;
public static final double FRONT_LEFT_ANGLE_OFFSET = -0.203369140625 + 0.5;

//Swerve Front Right
public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
public static final int FRONT_RIGHT_ANGLE_MOTOR_ID = 4;
public static final int FRONT_RIGHT_ANGLE_ENCODER_ID = 15;
public static final double FRONT_RIGHT_ANGLE_OFFSET = -0.140380859375;

//Swerve Back Left
public static final int BACK_LEFT_DRIVE_MOTOR_ID = 7;
public static final int BACK_LEFT_ANGLE_MOTOR_ID = 8;
public static final int BACK_LEFT_ANGLE_ENCODER_ID = 14;
public static final double BAKC_LEFT_ANGLE_OFFSET = 0.04443359375;

//Swerve Back Right
public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 5;
public static final int BACK_RIGHT_ANGLE_MOTOR_ID = 6;
public static final int BACK_RIGHT_ANGLE_ENCODER_ID = 16;
public static final double BACK_RIGHT_ANGLE_OFFSET = 0.42333984375; 

//Swerve General

public static final double ROBOT_BASE_LENGTH = Units.inchesToMeters(29.75); 
  public static final double ROBOT_BASE_WIDTH = Units.inchesToMeters(29.75);

  public static final double X_FROM_CENTER = ROBOT_BASE_LENGTH / 2;
  public static final double Y_FROM_CENTER = ROBOT_BASE_WIDTH / 2;

  public static final double FRONT_LEFT_X_LOCATION = X_FROM_CENTER;
  public static final double FRONT_LEFT_Y_LOCATION = Y_FROM_CENTER;

  public static final double FRONT_RIGHT_X_LOCATION = X_FROM_CENTER;
  public static final double FRONT_RIGHT_Y_LOCATION = -Y_FROM_CENTER;

  public static final double BACK_LEFT_X_LOCATION = -X_FROM_CENTER;
  public static final double BACK_LEFT_Y_LOCATION = Y_FROM_CENTER;

  public static final double BACK_RIGHT_X_LOCATION = -X_FROM_CENTER;
  public static final double BACK_RIGHT_Y_LOCATION = -Y_FROM_CENTER;

public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
    new Translation2d(FRONT_LEFT_X_LOCATION, FRONT_LEFT_Y_LOCATION),
    new Translation2d(FRONT_RIGHT_X_LOCATION, FRONT_RIGHT_Y_LOCATION),
    new Translation2d(BACK_LEFT_X_LOCATION, BACK_LEFT_Y_LOCATION),
    new Translation2d(BACK_RIGHT_X_LOCATION, BACK_RIGHT_Y_LOCATION));

public static final SensorDirectionValue ABSOLUTE_SENSOR_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;

    //Joysticks
    public static final double DEADBAND = 0.15;
    public static final int RIGHT_JOYSTICK_PORT = 2;
    public static final int MIDDLE_JOYSTICK_PORT = 1;
    public static final int LEFT_JOYSTICK_PORT = 0;

    public static final int RIGHT_JOYSTICK_BUTTON_TWELVE = 12;
    public static final int RIGHT_JOYSTICK_BUTTON_TWO = 2;
    public static final int INTAKE_BUTTON = 1;
    public static final int OUTTAKE_BUTTON = 3;
    public static final int RESET_GYRO_BUTTON = 11;
    public static final int REsET_ABSOLUTE_BUTTON = 12;

    public static final PhotonCamera PHOTON_CAMERA = new PhotonCamera("frontcam");

    public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(0.3048, 0, 0, new Rotation3d(0, 0, 0));
    public static final int FASTER_OUTTAKE_BUTTON = 2;

    public static class AprilTagConstants {
        public static final int TAG_TO_CHASE = 6;
        public static final Transform3d APRILTAG_LEFT = new Transform3d(new Translation3d(0.35, -0.10, 0.0), new Rotation3d(0, 0, Units.degreesToRadians(-170)));
        public static final Transform3d APRILTAG_RIGHT = new Transform3d(new Translation3d(0.56, 0.31, 0.0), new Rotation3d(0, 0, Units.degreesToRadians(-178)));;
        public static final Transform3d APRILTAG_MIDDLE = new Transform3d(new Translation3d(1.5, 0.0, 0.0), new Rotation3d(0, 0, Units.degreesToRadians(180)));;
    }
}