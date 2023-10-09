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

import static frc.robot.Constants.Swerve.*;

public class RobotContainer {
    // just for tuning, remove later
    private double setpoint = 0.0;

    private final Field2d field = new Field2d();

    private final CommandXboxController driver = new CommandXboxController(0);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final Swerve swerve = new Swerve(
        new SwerveModule(frontLeft),
        new SwerveModule(frontRight),
        new SwerveModule(backLeft),
        new SwerveModule(backRight),
        gyro, field
    );

    public RobotContainer() {
        swerve.setDefaultCommand(swerve.drive(
            driver::getLeftX,
            () -> kMaxSpeedMetersPerSecond * -driver.getLeftY(),
            driver::getRightX,
            // () -> 0, () -> 0, () -> Math.hypot(driver.getLeftX(), driver.getLeftY()) > 0.5 ? Math.atan2(driver.getLeftX(), driver.getLeftY()) : 0,
            true,
            Constants.kPeriodSeconds
        ));

        // SmartDashboard.putNumber("angleSetpointDeg", 0.0);
        // SmartDashboard.putNumber("veloSetpointDeg", 0.0);

        // swerve.setDefaultCommand(swerve.run(() -> {
        //     var angle = Math.atan2(driver.getLeftX(), driver.getLeftY());
        //     SmartDashboard.putNumber("controllerSetpointDeg", Math.toDegrees(angle));
        //     var state = new SwerveModuleState(
        //         SmartDashboard.getNumber("veloSetpointMetersPerSecond", 0.0),
        //         Rotation2d.fromRadians(angle));

        //     swerve.setModuleStates(new SwerveModuleState[] {
        //         state, state, state, state
        //     });
        // }));

        configureBindings();
    }

    private void configureBindings() {
        driver.a().onTrue(Commands.runOnce(() -> this.setpoint = SmartDashboard.getNumber("angleSetpointDeg", 0.0)));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}