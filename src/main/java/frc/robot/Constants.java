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


    public static class ElevatorConstants {
        public static final int ELEVATOR_MOTOR_ID = 9;
        public static final int LIMIT_SWITCH_ELEVATOR_TOP = 1;
        public static final double ELEVATOR_SPEED = .25;

        public static final int ELEVATOR_BUTTON_POSITION_ONE = 9;
        public static final int ELEVATOR_BUTTON_POSITION_TWO = 10;
        public static final int ELEVATOR_BUTTON_POSITION_THREE = 11;
        public static final int ELEVATOR_BUTTON_POSITION_FOUR = 12;

        public static final double ELEVATOR_DEADBAND = 5;
        public static final double ELEVATOR_POSITION_0 = 0;
        public static final double ELEVATOR_POSITION_1 = -40;
        public static final double ELEVATOR_POSITION_2 = -60;
        public static final double ELEVATOR_POSITION_3 = -90;
        public static final int ELEVATOR_UP_BUTTON = 6;
        public static final int ELEVATOR_DOWN_BUTTON = 4;
    }

    public static class ClimbConstants {
        public static final int CLIMB_MOTOR_ID = 13;
        public static final double CLIMB_SPEED = 0.5;
        public static final int CLIMB_FORWARD_BUTTON = 7;
        public static final int CLIMB_REVERSE_BUTTON = 8;

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
    public static final int RIGHT_JOYSTICK_BUTTON_TWELVE = 12;
    public static final int RIGHT_JOYSTICK_BUTTON_TWO = 2;
    public static final int INTAKE_BUTTON = 1;
    public static final int OUTTAKE_BUTTON = 3;
}
