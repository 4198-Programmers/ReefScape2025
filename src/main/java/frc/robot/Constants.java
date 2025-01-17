// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  //CANCoder Constants
  public static final double ABSOLUTE_ENCODER_DISCONTINUITY_POINT = 0.5;
  public static final SensorDirectionValue ABSOLUTE_ENCODER_SENSOR_DIRECTION = SensorDirectionValue.Clockwise_Positive;

  //Front Left Swerve Module Constants
  public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 7;
  public static final int Front_LEFT_ANGLE_MOTOR_ID = 8;
  public static final int FRONT_LEFT_CANCODER_ID = 7;
  public static final double FRONT_LEFT_ANGLE_OFFSET = 0;
  public static final int FRONT_LEFT_MODULE_NUMBER = 0;

  //Front Right Swerve Module Constants
  public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 1;
  public static final int Front_RIGHT_ANGLE_MOTOR_ID = 2;
  public static final int FRONT_RIGHT_CANCODER_ID = 2;
  public static final double FRONT_RIGHT_ANGLE_OFFSET = 0;
  public static final int FRONT_RIGHT_MODULE_NUMBER = 1;

  //Back Left Swerve Module Constants
  public static final int BACK_LEFT_DRIVE_MOTOR_ID = 10;
  public static final int BACK_LEFT_ANGLE_MOTOR_ID = 9;
  public static final int BACK_LEFT_CANCODER_ID = 9;
  public static final double BACK_LEFT_ANGLE_OFFSET = 0;
  public static final int BACK_LEFT_MODULE_NUMBER = 2;

  //Back Right Swerve Module Constants
  public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 4;
  public static final int BACK_RIGHT_ANGLE_MOTOR_ID = 3;
  public static final int BACK_RIGHT_CANCODER_ID = 3;
  public static final double BACK_RIGHT_ANGLE_OFFSET = 0;
  public static final int BACK_RIGHT_MODULE_NUMBER = 3;

  //DriveBase Lenghts
  public static final double ROBOT_BASE_LENGTH = 0.3;
  public static final double ROBOT_BASE_WIDTH = 0.295;

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

  //Swerve Drive
  public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
    new Translation2d(FRONT_LEFT_X_LOCATION, FRONT_LEFT_Y_LOCATION),
    new Translation2d(FRONT_RIGHT_X_LOCATION, FRONT_RIGHT_Y_LOCATION),
    new Translation2d(BACK_LEFT_X_LOCATION, BACK_LEFT_Y_LOCATION),
    new Translation2d(BACK_RIGHT_X_LOCATION, BACK_RIGHT_Y_LOCATION));
public static final double DEADBAND = 0.1;
public static final int RIGHT_JOYSTICK_PORT = 2;
public static final int MIDDLE_JOYSTICK_PORT = 1;
public static final int LEFT_JOYSTICK_PORT = 0;
}
