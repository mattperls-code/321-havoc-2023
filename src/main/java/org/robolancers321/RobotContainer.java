/* (C) Robolancers 2024 */
package org.robolancers321;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import javax.swing.plaf.TreeUI;

import org.robolancers321.subsystems.arm.Arm;
import org.robolancers321.subsystems.arm.commands.ManualMoveAnchor;
import org.robolancers321.subsystems.arm.commands.ManualMoveFloating;
import org.robolancers321.subsystems.arm.commands.MoveToSetpoint;
import org.robolancers321.subsystems.arm.commands.MoveToTunableSetpoint;
import org.robolancers321.subsystems.arm.commands.RunArm;

public class RobotContainer {
  private final Arm arm = new Arm();

  private final CommandXboxController driverController =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

  private final CommandXboxController manipulatorController =
      new CommandXboxController(Constants.OperatorConstants.kManipulatorControllerPort);

  private boolean isCubeMode = false;

  public RobotContainer() {
    this.arm.setDefaultCommand(new RunArm(arm));

    configureBindings();

    // try {
    //   String versionData = new VersionLoader().getVersionData().toString();
    //   System.out.println(versionData);
    //   SmartDashboard.putString("VERSION_DATA", versionData);

    // } catch (FileNotFoundException ex) {
    //   System.out.println("VERSIONING FILE NOT FOUND");
    //   SmartDashboard.putString("VERSION_DATA", "VERSIONING FILE NOT FOUND");
    // }
  }

  private void configureBindings() {
    driverController.rightBumper().whileTrue(new ManualMoveFloating(arm, true)); //up
    driverController.leftBumper().whileTrue(new ManualMoveFloating(arm, false)); //down
    driverController.leftTrigger().whileTrue(new ManualMoveAnchor(arm, true)); //up
    driverController.rightTrigger().whileTrue(new ManualMoveAnchor(arm, false)); //down

    // driverController
    //     .a()
    //     .onTrue(new MoveToSetpoint(arm, Constants.Arm.ArmSetpoints.SHELF, isCubeMode));
    // driverController
    //     .x()
    //     .onTrue(new MoveToSetpoint(arm, Constants.Arm.ArmSetpoints.MID, isCubeMode));
    // driverController
    //     .y()
    //     .onTrue(new MoveToSetpoint(arm, Constants.Arm.ArmSetpoints.HIGH, isCubeMode));

    SmartDashboard.putNumber("tunable dy", SmartDashboard.getNumber("tunable dy", 30.0));
    SmartDashboard.putNumber("tunable dz", SmartDashboard.getNumber("tunable dz", 20.0));

    driverController.b().onTrue(new MoveToTunableSetpoint(arm, 
      () -> SmartDashboard.getNumber("tunable dy", 0.0),
      () -> SmartDashboard.getNumber("tunable dz", 0.0)
    ));

    // driverController
    //     .start()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               if (isCubeMode == false) {
    //                 isCubeMode = true;
    //               } else {
    //                 isCubeMode = false;
    //               }
    //             }));

    SmartDashboard.putBoolean("isCubeMode", isCubeMode);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
