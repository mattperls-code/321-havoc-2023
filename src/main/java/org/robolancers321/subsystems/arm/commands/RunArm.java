/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.robolancers321.Constants;
import org.robolancers321.subsystems.arm.Arm;

public class RunArm extends CommandBase {
  private Arm arm;
  private double profileDT = 0.02;

  public RunArm(Arm arm) {
    this.arm = arm;

    addRequirements(arm);
  }

  @Override
  public void execute() {
    var anchorProfile =
        new TrapezoidProfile(
            Constants.Arm.Anchor.MP.ANCHOR_CONSTRAINTS,
            new TrapezoidProfile.State(arm.anchorSetpoint, 0),
            arm.getAnchorState());

    var floatingProfile =
        new TrapezoidProfile(
            Constants.Arm.Floating.MP.FLOATING_CONSTRAINTS,
            new TrapezoidProfile.State(arm.floatingSetpoint, 0),
            arm.getFloatingState());

    // update current state based on timestamp
    arm.anchorState = anchorProfile.calculate(profileDT);
    arm.floatingState = floatingProfile.calculate(profileDT);

    // on loop, these states are the new inital state for the profile

    double anchorFF =
        Constants.Arm.Anchor.FF.ANCHOR_FEEDFORWARD.calculate(
            Math.toRadians(arm.anchorState.position), 0);
    arm.setAnchorControllerReference(arm.anchorState.position, anchorFF);

    // double floatingFF =
    //     Constants.Arm.Floating.FF.FLOATING_FEEDFORWARD.calculate(
    //         Math.toRadians(arm.floatingState.position), 0);
    // arm.setFloatingControllerReference(arm.floatingState.position, floatingFF);

    
    // double anchorFF =
    //     Constants.Arm.Anchor.FF.ANCHOR_FEEDFORWARD.calculate(Math.toRadians(90 - arm.getAnchorAngle()), 0);
    // arm.setAnchorControllerReference(arm.anchorSetpoint, anchorFF);

    // SmartDashboard.putNumber("calculated ff", anchorFF);

    // // double floatingFF =
    // //     Constants.Arm.Floating.FF.FLOATING_FEEDFORWARD.calculate(
    // //         Math.toRadians(arm.floatingSetpoint), 0);
    // arm.setFloatingControllerReference(arm.floatingSetpoint, 0);

  }
}
