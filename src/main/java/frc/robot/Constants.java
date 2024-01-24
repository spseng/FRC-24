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

      // CAN IDs
      public static final int BR_STEER_CAN = 1;
      public static final int FR_STEER_CAN = 2;
      public static final int FL_STEER_CAN = 3;
      public static final int BL_STEER_CAN = 4;
      public static final int BR_DRIVE_CAN = 5;
      public static final int FR_DRIVE_CAN = 6;
      public static final int FL_DRIVE_CAN = 7;
      public static final int BL_DRIVE_CAN = 8;


      // Steering Offsets
      // public static final double BR_STEER_OFFSET =  0;
      // public static final double FR_STEER_OFFSET =  0;
      // public static final double FL_STEER_OFFSET =  0;
      // public static final double BL_STEER_OFFSET =  0;

      public static final double BR_STEER_OFFSET =  0.5;
      public static final double FR_STEER_OFFSET =  1  ;
      public static final double FL_STEER_OFFSET =  0.8;
      public static final double BL_STEER_OFFSET =  0.5;

      public static final double FULL_ROTATION = 2;
      public static final double RELATIVE_ENCODER_RATIO = 5.825;

      // Driver Settings
      public static final double DRIVE_SPEED = 0.35; // Zero to one
      public static final double TURN_SPEED = 0.15; // Zero to one
      public static final double TURN_SPEED_DRIVING = 1;
      public static final double JOYSTICK_DEAD_ZONE = 0.1; // Zero to one
      public static final double TRIGGER_DEAD_ZONE = 0.2; // Zero to one

      // TODO: Implement these:
        public static final double MAX_SPEED = 10; // Zero to one
        public static final double MAX_ACCELERATION = 0.5; // Zero to one
        public static final double MAX_TURING_SPEED = 0.5; // Zero to one
        public static final double MAX_TURNING_ACCELERATION = 0.5; // Zero to one

      // Robot Physical Constants
        public static final double WHEELBASE = 0.5; // Meters, distance between front and back
        public static final double TRACKWIDTH = 0.5; // Meters, distance between left and right


      // PID Constants
      // PID values will be carefully found through trial and error
      public static final double STEER_KP = 0.8;
      public static final double STEER_KI = 0.0005;
      public static final double STEER_KD = 0.00;

      public static final double TURNING_KP = 0.8;
      public static final double TURNING_KI = 0.0005;
      public static final double TURNING_KD = 0.00;
}
