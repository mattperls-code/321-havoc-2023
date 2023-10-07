package frc.robot.subsystems.swerve;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.Swerve.*;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final CANCoder turnEncoder;

    private final SparkMaxPIDController driveController;
    // private final SparkMaxPIDController turnController;
    private final PIDController turnController;

    public SwerveModule(ModuleConfig config) {
        this.driveMotor = new CANSparkMax(config.kDriveId, MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(config.kTurnId, MotorType.kBrushless);

        this.driveEncoder = driveMotor.getEncoder();
        this.turnEncoder = new CANCoder(config.kTurnEncoderId);

        this.driveController = driveMotor.getPIDController();
        // this.turnController = turnMotor.getPIDController();
        this.turnController = new PIDController(Turn.kP, Turn.kI, Turn.kD);

        configMotors();
        configEncoders(config.magOffsetDeg);
        configControllers();
    }

    public void update() {
        var output = turnController.calculate(
            turnEncoder.getAbsolutePosition(), Math.toRadians(SmartDashboard.getNumber("targetAngleDeg", 0)));
            // + SmartDashboard.getNumber("kTurnFF", Turn.kFF);
            
        SmartDashboard.putNumber("output", output);
        SmartDashboard.putNumber("setpoint", turnController.getSetpoint());

        turnMotor.set(MathUtil.clamp(output, -0.7, 0.7));
    }

    public void setDesiredState(SwerveModuleState state) {
        var optimizedState = SwerveModuleState.optimize(state, new Rotation2d(turnEncoder.getAbsolutePosition()));

        driveController.setReference(optimizedState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        // turnController.setReference(optimizedState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
        turnController.setSetpoint(optimizedState.angle.getRadians());

        SmartDashboard.putNumber("targetAngleDeg", optimizedState.angle.getDegrees());
        SmartDashboard.putNumber("currAngleDeg", Rotation2d.fromRadians(turnEncoder.getAbsolutePosition()).getDegrees());
    }

    public void setDrivePIDFCoeffs(double p, double i, double d, double f) {
        this.driveController.setP(p);
        this.driveController.setI(i);
        this.driveController.setD(d);
        this.driveController.setFF(f);
    }
    public void setTurnPIDFCoeffs(double p, double i, double d, double f) {
        this.turnController.setPID(p, i, d);
        // this.turnController.setP(p);
        // this.turnController.setI(i);
        // this.turnController.setD(d);
        // this.turnController.setFF(f);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            Rotation2d.fromRadians(turnEncoder.getAbsolutePosition())
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            Rotation2d.fromRadians(turnEncoder.getAbsolutePosition())
        );
    }

    private void configMotors() {
        driveMotor.setInverted(false);
        turnMotor.setInverted(false);

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
        var config = kCANCoderConfig;
        config.magnetOffsetDegrees = magOffsetDeg;

        turnEncoder.configAllSettings(config);

        driveEncoder.setVelocityConversionFactor(kRPMToMetersPerSecond);
    }

    private void configControllers() {
        setDrivePIDFCoeffs(Drive.kP, Drive.kI, Drive.kD, Drive.kFF);
        setTurnPIDFCoeffs(Turn.kP, Turn.kI, Turn.kD, Turn.kFF);

        turnController.enableContinuousInput(-Math.PI, Math.PI);
        // turnController.setPositionPIDWrappingEnabled(true);
        // turnController.setPositionPIDWrappingMaxInput(Math.PI);
        // turnController.setPositionPIDWrappingMinInput(-Math.PI);
    }
}