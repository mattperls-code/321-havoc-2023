/* (C) Robolancers 2024 */
package frc.robot.subsystems.arm;

import frc.robot.Constants;
import frc.robot.util.RelativePlane;

public class InverseArmKinematics {

  public static final class Angles {
    private final double anchor;
    private final double floating;

    public Angles(double anchor, double floating) {
      this.floating = floating;
      this.anchor = anchor;
    }

    public double getFloatingAngle() {
      return this.floating;
    }

    public double getAnchorAngle() {
      return this.anchor;
    }

    /*
        No setters needed
    */

  }

  private static double getReach(double y, double z) {
    return Math.sqrt(Math.pow(z, 2) + Math.pow(y, 2));
  }

  private static final double anchorLength = Constants.Arm.Anchor.kAnchorLength;

  private static final double floatLength = Constants.Arm.Floating.kFloatingLength;

  private final double dZ;
  private final double dY;

  // Length between base and floating

  private final double reach;

  public InverseArmKinematics(RelativePlane plane, double dZ, double dY) {
    this.dY = dY + plane.getYOffset();
    this.dZ = dZ;
    this.reach = getReach(this.dY, this.dZ);
  }

  private double calculateBeta() {
    double top = (Math.pow(this.reach, 2) - Math.pow(anchorLength, 2) - Math.pow(floatLength, 2));

    double bottom = (2 * anchorLength * floatLength);

    return Math.acos(top / bottom);
  }

  private static double calculateBeta(double rawZ, double rawY) {
    double reach = getReach(rawY, rawZ);
    double top = (Math.pow(reach, 2) - Math.pow(anchorLength, 2) - Math.pow(floatLength, 2));

    double bottom = (2 * anchorLength * floatLength);

    return Math.acos(top / bottom);
  }

  private double calculateAlpha(double beta) {
    return Math.asin(this.dY / this.reach) + Math.asin(floatLength * Math.sin(beta) / this.reach);
  }

  private static double calculateAlpha(double beta, double rawY, double rawZ) {
    double reach = getReach(rawY, rawZ);

    return Math.asin(rawY / reach) + Math.asin(floatLength * Math.sin(beta) / reach);
  }

  public Angles getAngles() {

    if (this.reach > floatLength + anchorLength) {
      return new Angles(Math.atan2(this.dY, this.dZ), 0);
    }

    double beta = this.calculateBeta();
    double alpha = this.calculateAlpha(beta);

    return new Angles(alpha, beta);
  }

  public static Angles getAnglesRaw(double rawZ, double rawY) {

    double reach = getReach(rawY, rawZ);

    if (reach > floatLength + anchorLength) {
      return new Angles(Math.atan2(rawY, rawZ), 0);
    }

    double beta = calculateBeta(rawZ, rawY);
    double alpha = calculateAlpha(beta, rawY, rawZ);

    return new Angles(alpha, beta);
  }
}
