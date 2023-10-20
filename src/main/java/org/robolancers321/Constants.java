/* (C) Robolancers 2024 */
package org.robolancers321;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.robolancers321.subsystems.arm.InverseArmKinematics;
import org.robolancers321.util.*;

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
    public static final int kManipulatorControllerPort = 1;
  }

  public static final class Arm {
    public static final class Anchor {
      public static final int kAnchorPort = 15;

      public static final boolean kEncoderInverted = false;
      public static final boolean kMotorInverted = true;
      public static final double kAnchorLength = 33; // in
      public static final double kZeroOffset = 145;
      public static final double kMinAngle = 0;
      public static final double kMaxAngle = 0;
      public static final double kNominalVoltage = 12.0;
      public static final boolean kEnableSoftLimit = false;
      public static final double kMaxOutput = 1; // going up
      public static final double kMinOutput = -1; // going down
      public static final int kCurrentLimit = 50; //
      public static final double kTolerance = 2.0; // error within 2 degrees

      public static final class PID {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final int kSlot = 0;
      }

      public static final class FF {
        // change to final when done tuning
        public static final double kS = 0;
        public static double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static ArmFeedforward ANCHOR_FEEDFORWARD = new ArmFeedforward(kS, kG, kV, kA);
      }

      public static final class MP {
        public static final double maxVel = 2.0;
        public static final double maxAccel = 1.0;
        public static final TrapezoidProfile.Constraints ANCHOR_CONSTRAINTS =
            new TrapezoidProfile.Constraints(maxVel, maxAccel);
      }

      public static final class Conversions {
        /*
        velocity - motorRot/s
        motorRot/s * deg/motorRot = deg/s

        Position - motorRot
        motorRot * mechRot/motorRot (gearRatio) * deg/mechRot = deg

        (endAngle - startAngle) / valueAtEndAngle)
         */

        public static final double kGearRatio = 64;
        public static final double kDegPerRot = (90.0 - (180.0 - 145.0)) / (8.333358764648438);
      }
      
      // 145deg -> -161.52581787109375
      // 90deg -> -108.32477569580078
    }

    public static final class Floating {
      public static final int kFloatingPort = 16;

      public static final boolean kEncoderInverted = false;
      public static final boolean kMotorInverted = true;
      public static final double kFloatingLength = 36; // in
      public static final double kZeroOffset = 0;
      public static final double kNominalVoltage = 12.0;
      public static final double kMinAngle = 0;
      public static final double kMaxAngle = 0;
      public static final boolean kEnableSoftLimit = false;
      public static final double kMaxOutput = 1; // going up
      public static final double kMinOutput = -1; // going down
      public static final int kCurrentLimit = 40;
      public static final double kTolerance = 2.0; // error within 2 degrees

      public static final class PID {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final int kSlot = 0;
      }

      public static final class FF {
        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final ArmFeedforward FLOATING_FEEDFORWARD =
            new ArmFeedforward(kS, kG, kV, kA);
      }

      public static final class MP {
        public static final double maxVel = 1.0;
        public static final double maxAccel = 1.0;
        public static final TrapezoidProfile.Constraints FLOATING_CONSTRAINTS =
            new TrapezoidProfile.Constraints(maxVel, maxAccel);
      }

      public static final class Conversions {
        public static final double kGearRatio = 25; // mechRot/motorRot. TODO check if correct or 1
        public static final double kDegPerRot = 1.0;
      }
    }

    public enum ArmSetpoints {
      /* From game manual, y is from carpet, z is from front of grid
      SHELF - 37.375 in high + 13 in from cone = 50.375
      MID - 34 in high, 22.75 in
      HIGH - 46 in high, 39.75 in
       */

      // SHELF(50.375, 0), 
      // MID(34, 22.75),
      // HIGH(46, 39.75),
      // TEST(0, 0);

      TEST(0, 0);

      private double anchor;
      private double floating;
      private double yOffset = 0; // from the ground

      ArmSetpoints(double y, double z) {
        InverseArmKinematics.Output angles = InverseArmKinematics.calculate(y - this.yOffset, z);

        this.anchor = y;
        this.floating = z;
      }

      public double getAnchor() {
        return this.anchor;
      }

      public double getFloating() {
        return this.floating;
      }
    }
  }
}
