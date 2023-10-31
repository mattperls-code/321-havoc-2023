/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.robolancers321.Constants;
import org.robolancers321.Constants.ArmSetpoints;
import org.robolancers321.subsystems.arm.Arm;
import org.robolancers321.util.MathUtils;

public class MoveFloating extends CommandBase {
  private Arm arm;
  private double floatingSetpoint;

  public MoveFloating(Arm arm, double floating) {
    this.arm = arm;
    this.floatingSetpoint = floating;
  }

  @Override
  public void initialize() {
    arm.setFloatingSetpoint(floatingSetpoint);
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
