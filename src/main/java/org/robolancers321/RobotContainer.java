/* (C) Robolancers 2024 */
package org.robolancers321;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.robolancers321.subsystems.arm.Arm;
import org.robolancers321.subsystems.arm.commands.RunArm;

public class RobotContainer {
  private final Arm arm = new Arm();

  private final CommandXboxController driverController =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

  private final CommandXboxController manipulatorController =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

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

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
