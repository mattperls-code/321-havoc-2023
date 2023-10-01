package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ChassisSpeedsUtil;
import frc.robot.util.SmartDashboardUtil;

import java.util.List;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Swerve.*;

public class Swerve extends SubsystemBase {
    private final Field2d field;
    private final List<SwerveModule> modules;

    private final AHRS gyro;

    private final SwerveDrivePoseEstimator poseEstimator;

    public Swerve(SwerveModule frontLeft, SwerveModule frontRight,
                  SwerveModule backLeft, SwerveModule backRight,
                  AHRS gyro, Field2d field) {
        this.modules = List.of(
            frontLeft,
            frontRight,
            backLeft,
            backRight
        );

        this.gyro = gyro;
        this.field = field;

        this.poseEstimator = new SwerveDrivePoseEstimator(
            kSwerveKinematics,
            gyro.getRotation2d(),
            getModulePositions(),
            new Pose2d()
        );
    }

    @Override
    public void periodic() {
        this.field.setRobotPose(poseEstimator.update(gyro.getRotation2d(), getModulePositions()));
        SmartDashboard.putData("Field", this.field);

        SmartDashboard.putNumber("frontLeftAngleDeg", this.modules.get(0).getPosition().angle.getDegrees());
        SmartDashboard.putNumber("frontRightAngleDeg", this.modules.get(1).getPosition().angle.getDegrees());
        SmartDashboard.putNumber("backLeftAngleDeg", this.modules.get(2).getPosition().angle.getDegrees());
        SmartDashboard.putNumber("backRightAngleDeg", this.modules.get(3).getPosition().angle.getDegrees());

        updateModulePIDFs();
    }

    public CommandBase drive(DoubleSupplier throttle, DoubleSupplier strafe, DoubleSupplier turn,
                             boolean fieldCentric, double periodSeconds) {
        return run(() -> drive(
            throttle.getAsDouble(),
            strafe.getAsDouble(),
            turn.getAsDouble(),
            fieldCentric,
            periodSeconds
        ));
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < modules.size(); i++) modules.get(i).setDesiredState(states[i]);
    }

    // this is just for tuning, remove after
    public void setModuleState(int index, SwerveModuleState state) {
        modules.get(index).setDesiredState(state);
    }

    public Pose2d getPose() {
        return this.poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        this.poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    private void drive(double throttle, double strafe, double turn,
                       boolean fieldCentric, double periodSeconds) {
        setModuleStates(
            statesFromChassisSpeeds(
                chassisSpeedsFromInputs(
                    throttle,
                    strafe,
                    turn,
                    fieldCentric,
                    periodSeconds)));
    }

    private ChassisSpeeds chassisSpeedsFromInputs(double throttle, double strafe, double turn,
                                                boolean fieldRelative, double periodSeconds) {
        return ChassisSpeedsUtil.discretize(
            fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                throttle, strafe, turn, gyro.getRotation2d())
            : new ChassisSpeeds(throttle, strafe, gyro.getRate()),
            periodSeconds);
    }

    private SwerveModuleState[] statesFromChassisSpeeds(ChassisSpeeds speeds) {
        final var states = kSwerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);
        return states;
    }

    private SwerveModulePosition[] getModulePositions() {
        return modules
            .stream()
            .map(SwerveModule::getPosition)
            .toArray(SwerveModulePosition[]::new);
    }

    private void updateModulePIDFs() {
        // final double driveP = SmartDashboardUtil.pollOrDefault("kDriveP", Drive.kP);
        // final double driveI = SmartDashboardUtil.pollOrDefault("kDriveI", Drive.kI);
        // final double driveD = SmartDashboardUtil.pollOrDefault("kDriveD", Drive.kD);
        // final double driveFF = SmartDashboardUtil.pollOrDefault("kDriveFF", Drive.kFF);

        final double turnP = SmartDashboardUtil.pollOrDefault("kTurnP", Turn.kP);
        final double turnI = SmartDashboardUtil.pollOrDefault("kTurnI", Turn.kI);
        final double turnD = SmartDashboardUtil.pollOrDefault("kTurnD", Turn.kD);
        final double turnFF = SmartDashboardUtil.pollOrDefault("kTurnFF", Turn.kFF);

        this.modules.forEach(module -> {
            // module.setDrivePIDFCoeffs(driveP, driveI, driveD, driveFF);
            module.setTurnPIDFCoeffs(turnP, turnI, turnD, turnFF);
        });
    }
}