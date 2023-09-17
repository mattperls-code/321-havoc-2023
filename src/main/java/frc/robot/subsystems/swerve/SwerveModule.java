package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import static frc.robot.Constants.Swerve.*;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final CANCoder turnEncoder;

    private final SparkMaxPIDController driveController;
    private final PIDController turnController;

    private SwerveModuleState desiredState;

    public SwerveModule(int driveMotorId, int turnMotorId, int turnEncoderId) {
        this.driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

        this.driveEncoder = driveMotor.getEncoder();
        this.turnEncoder = new CANCoder(turnEncoderId);

        this.driveController = driveMotor.getPIDController();
        this.turnController = new PIDController(
            kTurnP,
            kTurnI,
            kTurnD
        );

        configMotors();
        configEncoders();
        configControllers();
    }

    public void update() {
        double turnPIDOutput = turnController.calculate(turnEncoder.getPosition(), desiredState.angle.getRadians());

        turnMotor.setVoltage(turnPIDOutput);
    }

    public void setDesiredState(SwerveModuleState state) {
        var optimizedState = SwerveModuleState.optimize(state, new Rotation2d(turnEncoder.getPosition()));

        driveController.setReference(optimizedState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);

        this.desiredState = optimizedState;
    }

    public void setDrivePIDCoeffs(double p, double i, double d) {
        this.driveController.setP(p);
        this.driveController.setI(i);
        this.driveController.setD(d);
    }
    public void setTurnPIDCoeffs(double p, double i, double d) {
        this.turnController.setPID(p, i, d);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            new Rotation2d(turnEncoder.getPosition())
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
    }

    private void configEncoders() {
        driveEncoder.setInverted(false);

        var config = new CANCoderConfiguration();
        config.sensorCoefficient = 2 * Math.PI / 4096.0;
        config.unitString = "rad";
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;

        turnEncoder.configAllSettings(config);

        driveEncoder.setVelocityConversionFactor(kVelocityConversionFactor);
    }

    private void configControllers() {
        setDrivePIDCoeffs(kDriveP, kDriveI, kDriveD);
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }
}