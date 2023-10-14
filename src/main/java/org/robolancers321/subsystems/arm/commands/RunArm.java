/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm.commands;

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
    double customAnchorFF = arm.calculateCustomAnchorFeedforward();
    // PID
    double anchorFF =
        Constants.Arm.Anchor.FF.ANCHOR_FEEDFORWARD.calculate(Math.toRadians(arm.anchorSetpoint), 0);
    arm.setAnchorControllerReference(arm.anchorSetpoint, anchorFF);

    double customFloatingFF = arm.calculateCustomFloatingFeedforward();
    double floatingFF =
        Constants.Arm.Floating.FF.FLOATING_FEEDFORWARD.calculate(
            Math.toRadians(arm.floatingSetpoint), 0);
    arm.setFloatingControllerReference(arm.floatingSetpoint, floatingFF);

    // MOTION PROFILE

    // create motion profile to setpoint goal assuming joints are still. Feeding current angle in
    // the case the arm moves without being commanded ex. getting hit hard.
    // var anchorProfile =
    //     new TrapezoidProfile(
    //         Constants.Arm.Anchor.MP.ANCHOR_CONSTRAINTS,
    //         new TrapezoidProfile.State(arm.anchorSetpoint, 0),
    //         arm.getAnchorState());

    // var floatingProfile =
    //     new TrapezoidProfile(
    //         Constants.Arm.Floating.MP.FLOATING_CONSTRAINTS,
    //         new TrapezoidProfile.State(arm.floatingSetpoint, 0),
    //         arm.getFloatingState());

    // // update current state based on timestamp
    // arm.anchorState = anchorProfile.calculate(profileDT);
    // arm.floatingState = floatingProfile.calculate(profileDT);

    // // on loop, these states are the new inital state for the profile

    // double anchorFF =
    //     Constants.Arm.Anchor.FF.ANCHOR_FEEDFORWARD.calculate(
    //         Math.toRadians(arm.anchorState.position), 0);
    // arm.setAnchorControllerReference(arm.anchorState.position, anchorFF);

    // double floatingFF =
    //     Constants.Arm.Floating.FF.FLOATING_FEEDFORWARD.calculate(
    //         Math.toRadians(arm.floatingState.position), 0);
    // arm.setFloatingControllerReference(arm.floatingState.position, floatingFF);

  }
}
