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

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // TODO: replace with true robot mass
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

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

  public static final class ElevatorConstants {
    
    //unknown
    public static int ElevLeftCanID = 1;
    public static int ElevRightCanID = 2;
    
    public static final int TopLimitSwitchID = 3;
    public static final int BotLimitSwitchID = 4;

    public static final double kS = 1;
    public static final double kV = 2;
    public static final double kA = 2;

    public static final double kP = 0.8;
    public static final double kI = 0.2;
    public static final double kD = 0.1;

    public static final double kMaxV = 0.9;
    public static final double kMaxA = 0.9;

    public static final double kG = 0.4;
    
      
        
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
        public static final double L1ORSTOW = HeightToRotations(18);
        public static final double L2 = /*5*/ HeightToRotations(32);
        public static final double L3 = /*10*/HeightToRotations(47.625);
        public static final double L4 = /*15*/ HeightToRotations(72);
        public static final double REMOVEBALLL2 = /*4*/ HeightToRotations(32 + 8.125);
        public static final double REMOVEBALLL3 = /*8*/ HeightToRotations(47.625 + 8.125);
        public static final double SHOOTBALL = /*18*/ HeightToRotations(76 + 8.125);
      
    
  }

}
