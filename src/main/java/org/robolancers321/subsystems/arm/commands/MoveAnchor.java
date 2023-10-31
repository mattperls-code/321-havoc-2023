/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.robolancers321.Constants;
import org.robolancers321.Constants.ArmSetpoints;
import org.robolancers321.subsystems.arm.Arm;
import org.robolancers321.util.MathUtils;

public class MoveAnchor extends CommandBase {
  private Arm arm;

  private double anchorSetpoint;

  public MoveAnchor(Arm arm, double anchor) {
    this.arm = arm;
    this.anchorSetpoint = anchor;
  }

  @Override
  public void initialize() {
    arm.setFloatingSetpoint(anchorSetpoint);
  }

  @Override
  public boolean isFinished() {
      // return (MathUtils.epsilonEquals(
      //         anchorSetpoint, arm.getAnchorAngle(), Constants.Arm.Anchor.kTolerance)
      //     && MathUtils.epsilonEquals(
      //         floatingSetpoint, arm.getFloatingAngle(), Constants.Arm.Floating.kTolerance));
      return arm.getFloatingAtSetpoint();
  }
}
