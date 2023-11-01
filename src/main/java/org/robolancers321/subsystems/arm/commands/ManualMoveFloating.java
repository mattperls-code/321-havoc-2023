/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.robolancers321.subsystems.arm.Arm;

public class ManualMoveFloating extends CommandBase {
  private Arm arm;
  private final double degPerSec = 1;
  private double angleOffset = degPerSec / 1000 * 20;
  //  deg/s / 1000 = deg/ms * ms/loop = deg/loop

  private boolean reverse;

  public ManualMoveFloating(Arm arm, boolean reverse) {
    this.arm = arm;
    this.reverse = reverse;

    addRequirements(arm);
  }

  @Override
  public void execute() {
    if (reverse) {
      arm.setFloatingSetpoint(arm.getFloatingAngle() - angleOffset);
    } else {
      arm.setFloatingSetpoint(arm.getFloatingAngle() + angleOffset);
    }
  }
}

/*
  changing setpoint for match only
 */