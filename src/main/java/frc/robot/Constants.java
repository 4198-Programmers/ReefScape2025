// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
        public static final double ELEVATOR_SPEED = .25;

        public static final int ELEVATOR_BUTTON_POSITION_ONE = 9;
        public static final int ELEVATOR_BUTTON_POSITION_TWO = 10;
        public static final int ELEVATOR_BUTTON_POSITION_THREE = 11;
        public static final int ELEVATOR_BUTTON_POSITION_FOUR = 12;

        public static final double ELEVATOR_POSITION_0 = -5;
        public static final double ELEVATOR_POSITION_1 = -20;
        public static final double ELEVATOR_POSITION_2 = -50;
        public static final double ELEVATOR_POSITION_3 = -90;
        public static final int ELEVATOR_UP_BUTTON = 6;
        public static final int ELEVATOR_DOWN_BUTTON = 4;
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
    public static final int PRIMARY_JOINT_MOTOR_ID = 10; // The joint closest to elevator
    public static final double MANIPULATOR_MOTOR_DEADBAND = 0.15;
    public static final int ROTATING_MOTOR_ID = 11;
    public static final double MANIPULATOR_MOTOR_SPEED = 1.0;
    public static final double INTAKE_MOTOR_SPEED = 1.0;
    public static final int INTAKE_SENSOR_ID = 11;

    public static final int MANIPULATOR_ROTATE_BUTTON = 5;
  }
  //CANCoder Constants
  public static final double ABSOLUTE_ENCODER_DISCONTINUITY_POINT = 0.5; // :)
  public static final SensorDirectionValue ABSOLUTE_ENCODER_SENSOR_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;

  //Front Left Swerve Module Constants
  public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
  public static final int FRONT_LEFT_ANGLE_MOTOR_ID = 2;
  public static final int FRONT_LEFT_CANCODER_ID = 13;
  public static final double FRONT_LEFT_ANGLE_OFFSET = -0.192138671875;
  public static final int FRONT_LEFT_MODULE_NUMBER = 0;
  public static final boolean FRONT_LEFT_DRIVE_INVERT = true;
  public static final boolean FRONT_LEFT_ANGLE_INVERT = false;
  

  //Front Right Swerve Module Constants
  public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
  public static final int FRONT_RIGHT_ANGLE_MOTOR_ID = 4;
  public static final int FRONT_RIGHT_CANCODER_ID = 15;
  public static final double FRONT_RIGHT_ANGLE_OFFSET = -0.140380859375;
  public static final int FRONT_RIGHT_MODULE_NUMBER = 1;
  public static final boolean FRONT_RIGHT_DRIVE_INVERT = true;
  public static final boolean FRONT_RIGHT_ANGLE_INVERT = false;

  //Back Left Swerve Module Constants
  public static final int BACK_LEFT_DRIVE_MOTOR_ID = 7;
  public static final int BACK_LEFT_ANGLE_MOTOR_ID = 8;
  public static final int BACK_LEFT_CANCODER_ID = 14;
  public static final double BACK_LEFT_ANGLE_OFFSET = 0.04443359375;

  public static final int BACK_LEFT_MODULE_NUMBER = 2;
  public static final boolean BACK_LEFT_DRIVE_INVERT = false;
  public static final boolean BACK_LEFT_ANGLE_INVERT = false;
  

  //Back Right Swerve Module Constant
  public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 5;
  public static final int BACK_RIGHT_ANGLE_MOTOR_ID = 6;
  public static final int BACK_RIGHT_CANCODER_ID = 16;
  public static final double BACK_RIGHT_ANGLE_OFFSET = 0.42333984375;
  public static final int BACK_RIGHT_MODULE_NUMBER = 3;
  public static final boolean BACK_RIGHT_DRIVE_INVERT = false;
  public static final boolean BACK_RIGHT_ANGLE_INVERT = false;
  

  //DriveBase Lenghts
  public static final double ROBOT_BASE_LENGTH = 30;
  public static final double ROBOT_BASE_WIDTH = 30;

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

  //Swerve Drive Kinematics
  public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
    new Translation2d(FRONT_LEFT_X_LOCATION, FRONT_LEFT_Y_LOCATION),
    new Translation2d(FRONT_RIGHT_X_LOCATION, FRONT_RIGHT_Y_LOCATION),
    new Translation2d(BACK_LEFT_X_LOCATION, BACK_LEFT_Y_LOCATION),
    new Translation2d(BACK_RIGHT_X_LOCATION, BACK_RIGHT_Y_LOCATION));

    //Swerve General
    public static final double GEAR_RATIO = 12.8 / 1;
    public static final double DRIVE_GEAR_RATIO = 6.75 / 1;
    public static final double ANGLE_CONVERSION_FACTOR = 1 / 12.8;
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double DRIVE_POSITION_CONVERSION_FACTOR  = (WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO;
    public static final double DRIVE_VELOCITY_CONVERSION_FACOTR = DRIVE_POSITION_CONVERSION_FACTOR / 60;
    public static final double MAX_DRIVE_SPEED_MPS = Units.feetToMeters(15.1);

//Joysticks
public static final double DEADBAND = 0.1;
public static final int RIGHT_JOYSTICK_PORT = 2;
public static final int MIDDLE_JOYSTICK_PORT = 1;
public static final int LEFT_JOYSTICK_PORT = 0;


    public static final int RIGHT_JOYSTICK_BUTTON_TWELVE = 12;
    public static final int RIGHT_JOYSTICK_BUTTON_TWO = 2;
    public static final int INTAKE_BUTTON = 1;
    public static final int OUTTAKE_BUTTON = 3;
    public static final int RESET_GYRO_BUTTON = 11;
	  public static final int REsET_ABSOLUTE_BUTTON = 12;
}
