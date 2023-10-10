/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.robolancers321.Constants;
import org.robolancers321.subsystems.arm.Arm;

public class RunArm extends CommandBase {
  private Arm arm;

  public RunArm(Arm arm) {
    this.arm = arm;

    addRequirements(arm);
  }

  @Override
  public void execute() {
    double anchorFF =
        Constants.Arm.Anchor.FF.ANCHOR_FEEDFORWARD.calculate(Math.toRadians(arm.anchorSetpoint), 0);
    arm.setAnchorControllerReference(arm.anchorSetpoint, anchorFF);

    double floatingFF =
        Constants.Arm.Floating.FF.FLOATING_FEEDFORWARD.calculate(
            Math.toRadians(arm.floatingSetpoint), 0);
    arm.setFloatingControllerReference(arm.floatingSetpoint, floatingFF);

    // MOTION PROFILE
    // TrapezoidProfile.State anchorProfileState =
    // arm.periodicIO.anchorProfile.calculate(Timer.getFPGATimestamp() -
    // arm.periodicIO.anchorProfileStartTime);

    // TrapezoidProfile.State floatingProfileState =
    // arm.periodicIO.floatingProfile.calculate(Timer.getFPGATimestamp() -
    // arm.periodicIO.floatingProfileStartTime);

    // arm.periodicIO.anchorFF =
    // Constants.Arm.Anchor.FF.ANCHOR_FEEDFORWARD.calculate(Math.toRadians(anchorProfileState.position),
    // anchorProfileState.velocity);
    // arm.periodicIO.floatingFF =
    // Constants.Arm.Floating.FF.FLOATING_FEEDFORWARD.calculate(Math.toRadians(floatingProfileState.position),
    // floatingProfileState.velocity);

    // arm.floatingPIDController.setReference(
    //   floatingProfileState.position,
    //   ControlType.kPosition,
    //   Constants.Arm.Floating.PID.kSlot,
    //   arm.periodicIO.floatingFF,
    //   SparkMaxPIDController.ArbFFUnits.kVoltage);
    // arm.anchorPIDController.setReference(
    //   anchorProfileState.position,
    //   ControlType.kPosition,
    //   Constants.Arm.Anchor.PID.kSlot,
    //   arm.periodicIO.anchorFF,
    //   SparkMaxPIDController.ArbFFUnits.kVoltage);
  }
}
