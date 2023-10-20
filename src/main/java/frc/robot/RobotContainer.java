/* (C) Robolancers 2024 */
package frc.robot;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.OperatorConstants.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveModule;

public class RobotContainer {
  private final Field2d field = new Field2d();

  private final CommandXboxController driver = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private final Swerve swerve =
      new Swerve(
          new SwerveModule(frontLeft),
          new SwerveModule(frontRight),
          new SwerveModule(backLeft),
          new SwerveModule(backRight),
          gyro,
          field);

  public RobotContainer() {
    // swerve.setDefaultCommand(swerve.run(() -> swerve.setModuleStates(new SwerveModuleState(0, new Rotation2d()))));

    swerve.setDefaultCommand(
      swerve.drive(
        this::getThrottle,
        this::getStrafe,
        this::getTurn,
        true, kGearRatio));

    // for tuning speed or angle controllers individually
    // swerve.setDefaultCommand(
    //     swerve.run(
    //         () -> {
    //           // final var angle = Math.atan2(driver.getLeftX(), driver.getLeftY());
    //           // SmartDashboard.putNumber("controllerSetpointDeg", Math.toDegrees(angle));
    //           final var velo = driver.getLeftY() * kMaxSpeedMetersPerSecond;
    //           SmartDashboard.putNumber("controllerSetpointMetersPerSecond", velo);
    //           final var state = new SwerveModuleState(velo, Rotation2d.fromRadians(0));

    //           swerve.setModuleStates(state);
    //         }));

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private double getThrottle() {
    return kMaxSpeedMetersPerSecond * MathUtil.applyDeadband(driver.getLeftY(), kJoystickDeadband);
  }

  private double getStrafe() {
    return kMaxSpeedMetersPerSecond * MathUtil.applyDeadband(driver.getLeftX(), kJoystickDeadband);
  }

  private double getTurn() {
    return kMaxOmegaRadiansPerSecond * MathUtil.applyDeadband(driver.getRightX(), kJoystickDeadband);
  }
}