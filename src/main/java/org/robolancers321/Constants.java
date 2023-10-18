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
  }

  public static final class Arm {
    public static final class Anchor {
      public static final int kAnchorPort = 0;
      public static final int kAnchorEncoderPort = 0;

      public static final boolean kInverted = false;
      public static final double kAnchorLength = 0;
      public static final double kZeroPosition = 0;
      public static final double kMinAngle = Double.NEGATIVE_INFINITY;
      public static final double kMaxAngle = Double.POSITIVE_INFINITY;
      public static final double kNominalVoltage = 12.0;
      public static final boolean kEnableSoftLimit = false;
      public static final double kMaxOutput = 1; // going up
      public static final double kMinOutput = -1; // going down
      public static final int kCurrentLimit = 40; // 40 or 50 for best performance
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
         */

        public static final double kGearRatio = 64; //  TODO check if this is correct
        public static final double kDegPerRot = kGearRatio * 360;
      }
    }

    public static final class Floating {
      public static final int kFloatingPort = 0;
      public static final int kFloatingEncoderPort = 0;

      public static final boolean kInverted = false;
      public static final double kFloatingLength = 0;
      public static final double kZeroPosition = 0;
      public static final double kNominalVoltage = 12.0;
      public static final double kMinAngle = Double.NEGATIVE_INFINITY;
      public static final double kMaxAngle = Double.POSITIVE_INFINITY;
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
        public static final double kDegPerRot = kGearRatio * 360;
      }
    }

    public enum ArmSetpoints {
      /* Priority of setpoints
      1. HIGH
      2. CONTRACT honestly just give it zero positions no need to tune
      3. SHELF
      4. MID
      5. GROUND idk if the robot can even mechanically do this efficiently
       */

      TEST(0, 0);

      private double anchor;
      private double floating;

      ArmSetpoints(double y, double z) {
        InverseArmKinematics.Output angles = InverseArmKinematics.calculate(y, z);

        this.anchor = angles.anchor;
        this.floating = angles.floating;
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
