// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

  public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(0.2667, 0.2667),
    new Translation2d(0.2667, -0.2667),
    new Translation2d(-0.2667, 0.2667),
    new Translation2d(-0.2667, -0.2667)
  );

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
        GROUND(10),
        PROCESSOR(124.0556640625),
        GROUNDALG(85.0224609375),
        READY(107.015380859375),
        PICKUP(76.301513671875),
        L1(95),
        L2SCORE(14.3173828125),
        L2(62.855712890625),
        L3SCORE(100.355712890625),
        L3(124.666259765625),
        L4(218.253662109375),
        LOW(102), // 11.41455078125, 7.984863
        MID(29), // 27.5 
        HIGH(55), // 52
        LOWALG(95.2470703125), 
        HIGHALG(173.50126953125),
        BARGE(233);

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
          DEFAULT(-0.1806640625),
          PICKUP(1.82),
          SCORE(0.927),
          L1(1), //Placeholder, change when tuning
          L2_3(0.46630859375),
          FLIPL2_3(2*DEFAULT.getPosition()-L2_3.getPosition()),
          L4(0.402587890625),
          FLIPL4(2*DEFAULT.getPosition()-L4.getPosition()),
          GROUND_ALGAE(1.3),
          PROCESSOR(1.1),
          ALGAE(0.8),
          FLIPALGAE(2*DEFAULT.getPosition()-ALGAE.getPosition()),  
          BARGE(0.06), //Placeholder
          FLIPBARGE(2*DEFAULT.getPosition()-BARGE.getPosition()),
          LOLLYPOP(1.682373046875),
          MOVEPOSSIBLE(1.767),
          FLIPL1(2*DEFAULT.getPosition()-L1.getPosition());
          
          
          private final double position;
  
          ArmPositions(double position) {
              this.position = position;
          }
  
          public double getPosition() {
              return position;
          }
  
      }
    }
    public static class ApriltagConstants{
      public static final double yOffsetRight = 0.16;
      public static final double xOffsetRight = 0.53;
      public static final double yOffsetLeft = -0.17; //0.17
      public static final double xOffsetLeft = 0.53; //0.6

    }

    public static class IntakeConstants {
  
      // PID Constants
      public static final double kP = 0.7; // 0.675
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kIz = 0.0;
      public static final double kFF = 0.09;
      public static final double kG = 0;
      public static final double kA = 0.002;
  
      // Motion Magic Constants
      public static final double MAX_VELOCITY = 1600; // 10 inches per second
      public static final double MAX_ACCELERATION = 3000; // 20 inches per second squared
      public static final double MAX_JERK = 4000; // 60 inches per second cubed
      public static final double ALLOWABLE_ERROR = 0.5; // 0.5 inches
  
      public static final int CURRENT_LIMIT = 120;
      public static enum PivotPosition {
          GROUND(0), //16.754045486450195
          UP(-13.0054);

          private final double angle;
  
          PivotPosition(double angle) {
              this.angle = angle;
          }
  
          public double getAngle() {
              return angle;
          }
  
      }
    }
  }


