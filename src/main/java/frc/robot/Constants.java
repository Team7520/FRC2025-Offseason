// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ElevatorConstants {
    public static final int LIMIT_SWITCH_ID = 9;
    public static final int FRONT_MOTOR_ID = 31;
    public static final int BACK_MOTOR_ID = 32;
    public static final double SENSOR_TO_MECHANISM_RATIO = 3d/11d; // Units will be in inches
    public static final double MAX_HEIGHT = 60; // 60 inches

    // PID Constants
    public static final double kP = 0.7; // 0.675
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kIz = 0.0;
    public static final double kFF = 0.09;
    public static final double kG = 0;
    public static final double kA = 0.002;

    // Motion Magic Constants
    public static final double MAX_VELOCITY = 800; // 10 inches per second
    public static final double MAX_ACCELERATION = 2000; // 20 inches per second squared
    public static final double MAX_JERK = 4000; // 60 inches per second cubed
    public static final double ALLOWABLE_ERROR = 0.5; // 0.5 inches

    public static final int CURRENT_LIMIT = 120;
    public static enum ElevatorPosition {
        GROUND(0),
        GROUNDALG(79.355712890625),
        READY(100.308350),
        PICKUP(85.468018),
        L1(125.188965),
        LOW(102), // 11.41455078125, 7.984863
        MID(29), // 27.5 
        HIGH(55), // 52
        LOWALG(96.46435546875), 
        HIGHALG(162.46435546875);

        private final double height;

        ElevatorPosition(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }

    }
  }

    public static class ArmConstants {
      public static final double PIVOT_HORIZONTAL_OFFSET_ROT = 0.155029;

      public static final int ROLLER_CAN_ID = 24;
      public static final int LASER_CAN_ID = 20;
      public static final int PIVOT_CAN_ID = 33;

  
      // Motion Magic Constants
      public static final double MAX_VELOCITY = 800; // 10 inches per second
      public static final double MAX_ACCELERATION = 2000; // 20 inches per second squared
      public static final double MAX_JERK = 4000; // 60 inches per second cubed
      public static final double ALLOWABLE_ERROR = 0.5; // 0.5 inches
  
      public static final int CURRENT_LIMIT = 120;
      public static enum ArmPositions { //Remove unneeded ones later, for now just adding everything that comes to mind
          DEFAULT(0.1708984375),
          PICKUP(2.2),
          SCORE(0.927),
          L1(1.5), //Placeholder, change when tuning
          L2_3(1.01), //Placeholder, change when tuning
          L4(1.01), //Placeholder, change when tuning
          OPP_SCORE(-0.45),
          OPPL1(1.01), //Placeholder, change when tuning
          OPPL2_3(1.01), //Placeholder, change when tuning
          OPPL4(1.01), //Placeholder, change when tuning
          LOLLIPOP(1.01), //Placeholder, change when tuning
          GROUND_ALGAE(1.651611328125),
          LOW_ALGAE(1.01), //Placeholder
          HIGH_ALGAE(1.01), //Placeholder
          BARGE(1.01); //Placeholder
  
          
          private final double position;
  
          ArmPositions(double position) {
              this.position = position;
          }
  
          public double getPosition() {
              return position;
          }
  
      }
    }
  }


