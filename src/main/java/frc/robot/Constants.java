/* (C) Robolancers 2024 */
package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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

  public static class Arm {
    public static class Anchor{
      public static final int kAnchorPort = 0;
      public static final int kAnchorEncoderPort = 0;
      public static final boolean kInverted = false;
      public static final int kCurrentLimit = 60; //40 - 60
      public static final double kGearRatio = 1;

      /*
      velocity from encoder is rotations/s 
      rotations/s * (meters/rotations) = meters/s
      m/s what we need for motion profile
      */
      public static final double kdistancePerRotation = 0;

      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
  
      public static final double ks = 0;
      public static final double kg = 0;
      public static final double kv = 0;
      public static final double ka = 0;
      public static final ArmFeedforward ANCHOR_FEEDFORWARD = new ArmFeedforward(ks, kg, kv, ka);

      public static final double maxVelocity = 0;
      public static final double maxAcceleration = 0;
      public static final TrapezoidProfile.Constraints ANCHOR_CONSTRAINTS = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

 

      public static final double kMinAngle = Double.NEGATIVE_INFINITY;
      public static final double kMaxAngle = Double.POSITIVE_INFINITY;
      public static final boolean kEnableSoftLimit = false;
      public static final double kZeroPosition = 0;

      public static final double kMaxOutput = Double.POSITIVE_INFINITY;
      public static final double kMinOutput = Double.NEGATIVE_INFINITY;
    }

    public static class Floating{
      public static final int kFloatingPort = 0;
      public static final int kFloatingEncoderPort = 0;
      public static final boolean kInverted = false;
      public static final int kCurrentLimit = 60; //40 - 60
      public static final double kGearRatio = 1;
      public static final double kdistancePerRotation = 0; 

      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
  
      public static final double ks = 0;
      public static final double kg = 0;
      public static final double kv = 0;
      public static final double ka = 0;
      public static final ArmFeedforward FLOATING_FEEDFORWARD = new ArmFeedforward(ks, kg, kv, ka);

      public static final double maxVelocity = 0;
      public static final double maxAcceleration = 0;
      public static final TrapezoidProfile.Constraints FLOATING_CONSTRAINTS = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
  
      public static final double kMinAngle = Double.NEGATIVE_INFINITY;
      public static final double kMaxAngle = Double.POSITIVE_INFINITY;
      public static final boolean kEnableSoftLimit = false;
      public static final double kZeroPosition = 0;


      public static final double kMaxOutput = Double.POSITIVE_INFINITY;
      public static final double kMinOutput = Double.NEGATIVE_INFINITY;


    }
   
    

  }
}
