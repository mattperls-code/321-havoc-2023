/* (C) Robolancers 2024 */
package org.robolancers321;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.io.FileNotFoundException;
import org.robolancers321.subsystems.arm.Arm;
import org.robolancers321.subsystems.arm.commands.RunArm;
import org.robolancers321.util.VersionLoader;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private Arm arm = new Arm();
  private final CommandXboxController controller =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    this.arm.setDefaultCommand(new RunArm(arm));
    configureBindings();

    try {
      String versionData = new VersionLoader().getVersionData().toString();
      System.out.println(versionData);
      SmartDashboard.putString("VERSION_DATA", versionData);

    } catch (FileNotFoundException ex) {
      System.out.println("VERSIONING FILE NOT FOUND");
      SmartDashboard.putString("VERSION_DATA", "VERSIONING FILE NOT FOUND");
    }
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
