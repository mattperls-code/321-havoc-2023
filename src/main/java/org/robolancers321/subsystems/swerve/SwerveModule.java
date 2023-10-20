/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.swerve;

import static org.robolancers321.Constants.Swerve.*;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  public final String id;

  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;

  private final RelativeEncoder driveEncoder;
  private final CANCoder turnEncoder;

  private final SparkMaxPIDController driveController;
  private final PIDController turnController;

  public SwerveModule(ModuleConfig config) {
    this.id = config.id;

    this.driveMotor = new CANSparkMax(config.kDriveId, MotorType.kBrushless);
    this.turnMotor = new CANSparkMax(config.kTurnId, MotorType.kBrushless);

    this.driveEncoder = driveMotor.getEncoder();
    this.turnEncoder = new CANCoder(config.kTurnEncoderId);

    this.driveController = driveMotor.getPIDController();
    this.turnController = new PIDController(Turn.kP, Turn.kI, Turn.kD);

    configMotors(config.driveIsInverted, config.turnIsInverted);
    configEncoders(config.magOffsetDeg);
    configControllers();
  }

  public void updateTurnOutput() {
    final var output = turnController.calculate(turnEncoder.getAbsolutePosition());

    turnMotor.set(-MathUtil.clamp(output, -1.0, 1.0));
  }

  public void setDesiredState(SwerveModuleState state) {
    final var optimizedState =
        SwerveModuleState.optimize(state, new Rotation2d(turnEncoder.getAbsolutePosition()));

    driveController.setReference(
        optimizedState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turnController.setSetpoint(optimizedState.angle.getRadians());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), Rotation2d.fromRadians(turnEncoder.getAbsolutePosition()));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity() * kRPMToMetersPerSecond,
        Rotation2d.fromRadians(turnEncoder.getAbsolutePosition()));
  }

  private void configMotors(boolean driveIsInverted, boolean turnIsInverted) {
    driveMotor.setInverted(driveIsInverted);
    turnMotor.setInverted(turnIsInverted);

    driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    driveMotor.setSmartCurrentLimit(40);
    turnMotor.setSmartCurrentLimit(40);

    driveMotor.enableVoltageCompensation(12);
    turnMotor.enableVoltageCompensation(12);

    driveMotor.burnFlash();
    turnMotor.burnFlash();
  }

  private void configEncoders(double magOffsetDeg) {
    final var config = kCANCoderConfig;
    config.magnetOffsetDegrees = magOffsetDeg;

    turnEncoder.configAllSettings(config);
  }

  private void configControllers() {
    this.driveController.setP(Drive.kP);
    this.driveController.setI(Drive.kI);
    this.driveController.setD(Drive.kD);
    this.driveController.setFF(Drive.kFF);

    this.turnController.setPID(Turn.kP, Turn.kI, Turn.kD);

    this.turnController.enableContinuousInput(-Math.PI, Math.PI);
  }
}
