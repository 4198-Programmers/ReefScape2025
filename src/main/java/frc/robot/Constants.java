// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static class ManipulatorConstants {
    public static final int INTAKE_MOTOR_ID = 1;
    public static final int PRIMARY_JOINT_MOTOR_ID = 7; // The joint closest to elevator
    public static final double MANIPULATOR_MOTOR_DEADBAND = 70;
    public static final int ROTATING_MOTOR_ID = 6;
    public static final double MANIPULATOR_MOTOR_SPEED = 0.25;
    public static final double INTAKE_MOTOR_SPEED = 0.25;
  }
    public static final int RIGHT_JOYSTICK_ID = 2;
    public static final int RIGHT_JOYSTICK_BUTTON_TWELVE = 12;
    public static final int RIGHT_JOYSTICK_BUTTON_TWO = 2;
    public static final int INTAKE_BUTTON = 1;
    public static final int OUTTAKE_BUTTON = 3;
}
