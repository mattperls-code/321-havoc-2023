// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

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
import frc.robot.util.SmartDashboardUtil;

import static frc.robot.Constants.Swerve.*;

public class RobotContainer {

    private final Field2d field = new Field2d();

    private final CommandXboxController driver = new CommandXboxController(0);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final Swerve swerve = new Swerve(
        new SwerveModule(frontLeft.kDriveId, frontLeft.kTurnId, frontLeft.kTurnEncoderId),
        new SwerveModule(frontRight.kDriveId, frontRight.kTurnId, frontRight.kTurnEncoderId),
        new SwerveModule(backLeft.kDriveId, backLeft.kTurnId, backLeft.kTurnEncoderId),
        new SwerveModule(backRight.kDriveId, backRight.kTurnId, backRight.kTurnEncoderId),
        gyro, field
    );

    public RobotContainer() {
        // swerve.setDefaultCommand(swerve.drive(
        //     driver::getLeftX,
        //     () -> -driver.getLeftY(),
        //     driver::getRightX,
        //     true,
        //     Constants.kPeriodSeconds
        // ));

        // SmartDashboardUtil.pollOrDefault("veloSetpointMetersPerSecond", 0.0);
        // SmartDashboardUtil.pollOrDefault("angleSetpointDeg", 0.0);

        SmartDashboard.putNumber("veloSetpointMetersPerSecond", 0.0);
        SmartDashboard.putNumber("angleSetpointDeg", 0.0);

        swerve.setDefaultCommand(swerve.run(() ->
            swerve.setModuleState(0, new SwerveModuleState(
                SmartDashboardUtil.pollOrDefault("veloSetpointMetersPerSecond", 0.0),
                Rotation2d.fromDegrees(SmartDashboardUtil.pollOrDefault("angleSetpointDeg", 0.0))
        ))));

        configureBindings();
    }

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}