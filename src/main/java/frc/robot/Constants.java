// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // TODO: replace with true robot mass
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    // Max velocities and accelerations for auto
    public static final double kMaxAutoVelocity = 5.0; 
    public static final double kMaxAutoAcceleration = 3.7; 

    // Max velocities and accelerations for teleop alignment
    public static final double kMaxAlignmentVelocity = 2.1;  
    public static final double kMaxAlignmentAcceleration = 1.3; // formerly 1.8
    public static final double kMaxOmega = Math.toRadians(270);
    public static final double kMaxAlpha = Math.toRadians(360);

    // Tunable constants
    public static final double kS = 0.185;
    public static final double kV = 1.866;
    public static final double kA = 0.159;

    public static final double kP_translation = 2.0;  
    public static final double kI_translation = 0; 
    public static final double kD_translation = 0.15; 

    public static final double kP_rotation = 3.0;
    public static final double kI_rotation = 0;
    public static final double kD_rotation = 0.15;

    public static final double kStdvX = 0.08;
    public static final double kStdvY = 0.08;
    public static final double kStdvTheta = 999999;
  }

  
  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class ElevatorConstants {
    
    //unknown
    public static int ElevLeftCanID = 15;
    public static int ElevRightCanID = 16;
    
    public static final int TopLimitSwitchID = 3;
    public static final int BotLimitSwitchID = 4;
    

    public static final double kS = 1;
    public static final double kV = 2;
    public static final double kA = 2;

    public static final double kP = 0.8;
    public static final double kI = 0.2;
    public static final double kD = 0.1;

    public static final double kMaxV = 3;
    public static final double kMaxA = 3;

    public static final double kG = 0.4;

    public static final double kElevatorTolerance = 1.0;
    
      
        
        public static enum ElevState {
          L2BALL(HeightToRotations(32 + 8.125)),
          L3BALL(HeightToRotations(47.625 + 8.125)),
          SHOOTBALL(HeightToRotations(76 + 8.125)),
          STOW(HeightToRotations(18)),
          GROUNDBALL(HeightToRotations(20));
        

          public final double elevatorSetPoint;
          
          private ElevState(double elevatorSetpoint) {
            this.elevatorSetPoint = elevatorSetpoint;
          }

          public double getValue() {
            return elevatorSetPoint;
          }

        }

        public static double HeightToRotations(double TargetHeight) {
          return ((TargetHeight-WristHeightOffGround)/NumInPerRot);
        } 

        //placeholder
        public static final double NumInPerRot = 13.4962820398;
        public static final double WristHeightOffGround = 17;
        //need to be tested


        public static final double L2 = 5;///*5*/ HeightToRotations(32);
        public static final double L3 = 10;///*10*/HeightToRotations(47.625);
        public static final double L4 = 15;///*15*/ HeightToRotations(72);
        public static final double REMOVEBALLL2 = 3;///*4*/ HeightToRotations(32 + 8.125);
        public static final double REMOVEBALLL3 = 7;///*8*/ HeightToRotations(47.625 + 8.125);
        public static final double SHOOTBALL = 14;///*18*/ HeightToRotations(76 + 8.125);
        public static final double L2BALL = 3;//HeightToRotations(32 + 8.125);
        public static final double L3BALL = 5;//HeightToRotations(47.625 + 8.125);
        public static final double STOW = 0.2;//HeightToRotations(18);
        public static final double GROUNDBALL = 0.2;//HeightToRotations(20);
    
  }

  public static final class ArmConstants
  {
    /** CAN IDs */
    public static final int MOTOR_ID = 40;      // TODO: set correct ID
    public static final int ENCODER_ID = 0;    // TODO: set correct ID

    /** Characterization Gains */
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kG = 0.0;

    /** PID Gains */
    public static final double kP = 0.8;
    public static final double kI = 0.2;
    public static final double kD = 0.1;

    /** Motion Constraints */
    public static final double kMaxV = 4.0; // m/s
    public static final double kMaxA = 5.0; // m/s^2

    /** Arm PID Gains */
    public static final double ARM_KP = 0.5;
    public static final double ARM_KI = 0.01;
    public static final double ARM_KD = 0.0;
    public static final double ARM_KS = 0.1;
    public static final double ARM_KG = 0.0;
    public static final double ARM_KV = 0.0;
    public static final double ARM_KA = 0.0;

    /** Arm Geometry */
    public static final double MOTOR_GEAR_RATIO = 108.0; // Motor reduction ratio
    public static final double ENCODER_GEAR_RATIO = 81.0; // Encoder reduction ratio
    public static final double ARM_LENGTH_METERS = 0.5; // TODO: measure (m)
    public static final double ARM_MASS_KG = 5.0;       // TODO: measure (kg)
    public static final double MAX_ANGLE_DEG = 0.0;
    public static final double MIN_ANGLE_DEG = -70.0;
    

    /** Gravity constant */
    public static final double GRAVITY = 9.81; // m/s^2

    /** Arm Setpoints (Degrees) */
    public static final double STOW_POSITION_DEG = 0.0;
    public static final double PICKUP_POSITION_DEG = -88;
    public static final double REEF_POSITION_DEG_low = -11; //11.0+90.0;
    public static final double REEF_POSITION_DEG_high = -11; //11.0+90.0;
    public static final double BARGE = 0.0; //31.0+90.0;
    public static final double PROCESSOR_DEG = 0.0;
    

    // Absolute encoder setup
  public static final int ABS_ENCODER_DIO_PORT = 0;   // change to your wiring
  public static final double ABS_ENCODER_OFFSET_DEG = 0.0; // tune so stow = 0°
  public static final boolean ABS_ENCODER_REVERSED = false; 


    /** Arm Setpoints (Radians) */
    public static final double STOW_POSITION_RAD = Math.toRadians(STOW_POSITION_DEG);
    public static final double PICKUP_POSITION_RAD = Math.toRadians(PICKUP_POSITION_DEG);
    public static final double REEF_POSITION_RAD_high = Math.toRadians(REEF_POSITION_DEG_high);
    public static final double REEF_POSITION_RAD_low = Math.toRadians(REEF_POSITION_DEG_low);

    /** Arm State Enum */
    public enum ArmState {
      STOW(STOW_POSITION_DEG),
      PICKUP(PICKUP_POSITION_DEG),
      REEF_high(REEF_POSITION_DEG_high),
      REEF_low(REEF_POSITION_DEG_low);

      public final double angleDeg;

      ArmState(double angleDeg) {
        this.angleDeg = angleDeg;
      }
    }
  }

  public static final class FieldConstants{
    // AUTO 
    // Left and right are viewed from the DS.
    public static final double RED_X = 10.402;
    public static final double BLUE_X = 7.164;

    public static final Pose2d FAR_RIGHT = new Pose2d(RED_X, 7.145, Rotation2d.fromDegrees(-180));
    public static final Pose2d FAR_LEFT = new Pose2d(RED_X, 1.08, Rotation2d.fromDegrees(-180));
    public static final Pose2d RIGHT = new Pose2d(RED_X, 5.645, Rotation2d.fromDegrees(-180));
    public static final Pose2d LEFT = new Pose2d(RED_X, 2.636, Rotation2d.fromDegrees(-180));
    public static final Pose2d CENTER_RED = new Pose2d(RED_X, 4.0, Rotation2d.fromDegrees(-180));

    public static final Pose2d FAR_LEFT_BLUE = new Pose2d(BLUE_X, 6.92, Rotation2d.fromDegrees(0));
    public static final Pose2d FAR_RIGHT_BLUE = new Pose2d(BLUE_X, 1, Rotation2d.fromDegrees(0));
    public static final Pose2d LEFT_BLUE = new Pose2d(BLUE_X, 5.645, Rotation2d.fromDegrees(0));
    public static final Pose2d RIGHT_BLUE = new Pose2d(BLUE_X, 2.636, Rotation2d.fromDegrees(0));
    public static final Pose2d CENTER_BLUE = new Pose2d(BLUE_X, 4.0, Rotation2d.fromDegrees(0));

    public static final double Elevator_top_algae = 8.15;
    public static final double Wrist_top_algae = 148.5;
    public static final double Elevator_low_algae = 1.51;
    public static final double Wrist_low_algae = 148.5;

    public static Pose2d getFarRight(DriverStation.Alliance alliance){
      return alliance == DriverStation.Alliance.Red ? FAR_RIGHT : FAR_RIGHT_BLUE;
    }

    public static Pose2d getFarLeft(DriverStation.Alliance alliance){
      return alliance == DriverStation.Alliance.Red ? FAR_LEFT : FAR_LEFT_BLUE;
    }

    public static Pose2d getRight(DriverStation.Alliance alliance){
      return alliance == DriverStation.Alliance.Red ? RIGHT : RIGHT_BLUE;
    }

    public static Pose2d getLeft(DriverStation.Alliance alliance){
      return alliance == DriverStation.Alliance.Red ? LEFT : LEFT_BLUE;
    }

    public static Pose2d getCenter(DriverStation.Alliance alliance){
      return alliance == DriverStation.Alliance.Red ? CENTER_RED : CENTER_BLUE;
    }
  }
}
