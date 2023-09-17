// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveModule;

import static frc.robot.Constants.Swerve.*;

public class RobotContainer {

    private final Field2d field = new Field2d();

    private final CommandXboxController driver = new CommandXboxController(0);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final Swerve swerve = new Swerve(
        new SwerveModule(kFrontLeftDriveId, kFrontLeftTurnId, kFrontLeftTurnEncoderId),
        new SwerveModule(kFrontRightDriveId, kFrontRightTurnId, kFrontRightTurnEncoderId),
        new SwerveModule(kBackLeftDriveId, kBackLeftTurnId, kBackLeftTurnEncoderId),
        new SwerveModule(kBackRightDriveId, kBackRightTurnId, kBackRightTurnEncoderId),
        gyro, field
    );

    public RobotContainer() {
         swerve.setDefaultCommand(swerve.drive(
             driver::getLeftX,
             () -> -driver.getLeftY(),
             driver::getRightX,
             true,
             Constants.kPeriodSeconds
         ));

        configureBindings();
    }

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}