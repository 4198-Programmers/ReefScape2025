// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int RIGHT_JOYSTICK_ID = 2;
  public static final int RIGHT_JOYSTICK_BUTTON_THREE = 3;
  public static final int RIGHT_JOYSTICK_BUTTON_FIVE = 5;

  public static class ElevatorConstants {
    public static final int ELEVATOR_MOTOR_ID = 9;
    public static final int LIMIT_SWITCH_ELEVATOR_TOP = 1;
    public static final double ELEVATOR_SPEED = 1.0;
  }

  public static final int CLIMB_MOTOR_ID = 13;
  public static final int JOYSTICK_RIGHT_ID = 2;
  public static final int JOYSTICK_MIDDLE_ID = 1;
  public static final int JOYSTICK_LEFT_ID = 0;

  public static final double CLIMB_SPEED = 0.5;
}
