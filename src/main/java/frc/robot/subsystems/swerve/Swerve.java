package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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

        updateModules();
    }

    public CommandBase drive(DoubleSupplier xTrans, DoubleSupplier yTrans, DoubleSupplier turn,
                             boolean fieldCentric, double periodSeconds) {
        return run(() -> drive(
            xTrans.getAsDouble(),
            yTrans.getAsDouble(),
            turn.getAsDouble(),
            fieldCentric,
            periodSeconds
        ));
    }

    private void drive(double throttle, double strafe, double turn,
                       boolean fieldCentric, double periodSeconds) {
         final var states = kSwerveKinematics.toSwerveModuleStates(
            inputsToChassisSpeeds(
                throttle,
                strafe,
                turn,
                fieldCentric,
                periodSeconds)
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);

        for (int i = 0; i < modules.size(); i++) modules.get(i).setDesiredState(states[i]);
    }

    private ChassisSpeeds inputsToChassisSpeeds(double throttle, double strafe, double turn,
                                                boolean fieldRelative, double periodSeconds) {
        return ChassisSpeedsUtil.discretize(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
    throttle, strafe, turn, gyro.getRotation2d())
            : new ChassisSpeeds(throttle, strafe, gyro.getRate()),
            periodSeconds);
    }

    private SwerveModulePosition[] getModulePositions() {
        return modules
            .stream()
            .map(SwerveModule::getPosition)
            .toArray(SwerveModulePosition[]::new);
    }

    private void updateModules() {
        final double driveP = SmartDashboardUtil.pollOrDefault("kDriveP", kDriveP);
        final double driveI = SmartDashboardUtil.pollOrDefault("kDriveP", kDriveI);
        final double driveD = SmartDashboardUtil.pollOrDefault("kDriveD", kDriveD);

        final double turnP = SmartDashboardUtil.pollOrDefault("kTurnP", kTurnP);
        final double turnI = SmartDashboardUtil.pollOrDefault("kTurnI", kTurnI);
        final double turnD = SmartDashboardUtil.pollOrDefault("kTurnD", kTurnD);

        this.modules.forEach(module -> {
            module.setDrivePIDCoeffs(driveP, driveI, driveD);
            module.setTurnPIDCoeffs(turnP, turnI, turnD);
            module.update();
        });
    }
}