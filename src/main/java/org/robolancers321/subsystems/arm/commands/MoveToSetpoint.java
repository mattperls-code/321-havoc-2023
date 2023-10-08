/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.robolancers321.Constants;
import org.robolancers321.Constants.Arm.ArmSetpoints;
import org.robolancers321.subsystems.arm.Arm;

public class MoveToSetpoint extends CommandBase {

  private double anchorPosSetpoint;
  private double floatingPosSetpoint;
  Arm arm;

  public MoveToSetpoint(Arm arm, ArmSetpoints setpoint) {
    this.arm = arm;
    this.anchorPosSetpoint = setpoint.getAnchor();
    this.floatingPosSetpoint = setpoint.getFloating();

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.periodicIO.anchorPosSetpoint = anchorPosSetpoint;
    arm.periodicIO.floatingPosSetpoint = floatingPosSetpoint;

    // MOTION PROFILE
    // create new MP with setpoints & get startTime
    arm.periodicIO.anchorProfile =
        new TrapezoidProfile(
            Constants.Arm.Anchor.MP.ANCHOR_CONSTRAINTS,
            new TrapezoidProfile.State(anchorPosSetpoint, 0),
            new TrapezoidProfile.State(arm.getAnchorAngle(), arm.getAnchorVelocity()));

    arm.periodicIO.floatingProfile =
        new TrapezoidProfile(
            Constants.Arm.Floating.MP.FLOATING_CONSTRAINTS,
            new TrapezoidProfile.State(floatingPosSetpoint, 0),
            new TrapezoidProfile.State(arm.getFloatingAngle(), arm.getFloatingVelocity()));

    arm.periodicIO.anchorProfileStartTime = Timer.getFPGATimestamp();
    arm.periodicIO.floatingProfileStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public boolean isFinished() {
    return arm.getAnchorAngle() == anchorPosSetpoint
        && arm.getFloatingAngle() == floatingPosSetpoint;
  }
}
