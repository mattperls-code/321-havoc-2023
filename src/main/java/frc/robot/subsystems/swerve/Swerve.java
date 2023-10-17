/* (C) Robolancers 2024 */
package frc.robot.subsystems.swerve;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.OperatorConstants.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ChassisSpeedsUtil;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {
	private final Field2d field;
	private final List<SwerveModule> modules;

	private final AHRS gyro;

	private final SwerveDrivePoseEstimator poseEstimator;

	public Swerve(
			SwerveModule frontLeft,
			SwerveModule frontRight,
			SwerveModule backLeft,
			SwerveModule backRight,
			AHRS gyro,
			Field2d field) {
		this.modules = List.of(frontLeft, frontRight, backLeft, backRight);

		this.gyro = gyro;
		this.field = field;

		this.poseEstimator = new SwerveDrivePoseEstimator(
				kSwerveKinematics, gyro.getRotation2d(), getModulePositions(), new Pose2d());

		initModulePIDF();

		gyro.zeroYaw();
		// gyro.setAngleAdjustment(90);
	}

	@Override
	public void periodic() {
		this.field.setRobotPose(poseEstimator.update(gyro.getRotation2d(), getModulePositions()));
		SmartDashboard.putData("Field", this.field);

		SmartDashboard.putNumber("yaw", gyro.getRotation2d().getDegrees());

		modules.forEach(
			module -> {
				SmartDashboard.putNumber(module.id + " drive motor position", module.getPosition().distanceMeters);
				module.setDrivePIDFCoeffs(
					SmartDashboard.getNumber("kDriveP", Drive.kP),
					SmartDashboard.getNumber("kDriveI", Drive.kI),
					SmartDashboard.getNumber("kDriveD", Drive.kD),
					SmartDashboard.getNumber("kDriveFF", Drive.kFF));
				module.setTurnPIDCoeffs(
					SmartDashboard.getNumber("kTurnP", Turn.kP),
					SmartDashboard.getNumber("kTurnI", Turn.kI),
					SmartDashboard.getNumber("kTurnD", Turn.kD));

				module.updateTurnOutput();
				SmartDashboard.putNumber(
						module.id + " speedMetersPerSecond", module.getState().speedMetersPerSecond);
				SmartDashboard.putNumber(module.id + " angleDeg", module.getState().angle.getDegrees());
			});
	}

	public CommandBase drive(
		DoubleSupplier throttle,
		DoubleSupplier strafe,
		DoubleSupplier turn,
		boolean fieldCentric,
		double periodSeconds) {
		return run(() -> drive(
			throttle.getAsDouble(),
			strafe.getAsDouble(),
			turn.getAsDouble(),
			fieldCentric, periodSeconds));
	}

	public void setModuleStates(SwerveModuleState state) {
		setModuleStates(
				Collections.nCopies(modules.size(), state)
						.toArray(new SwerveModuleState[modules.size()]));
	}

	public void setModuleStates(SwerveModuleState[] states) {
		for (int i = 0; i < modules.size(); i++)
			modules.get(i).setDesiredState(states[i]);
	}

	public Pose2d getPose() {
		return this.poseEstimator.getEstimatedPosition();
	}

	public void resetPose(Pose2d pose) {
		this.poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
	}

	public void drive(
			double throttle, double strafe, double turn, boolean fieldRelative, double periodSeconds) {
		final var speeds = // ChassisSpeedsUtil.discretize(
			fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(throttle, strafe, turn, gyro.getRotation2d())
				: new ChassisSpeeds(throttle, strafe, turn);
				// , periodSeconds);

		drive(speeds);
	}

	public void drive(ChassisSpeeds speeds) {
		final var states = kSwerveKinematics.toSwerveModuleStates(speeds);

		SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);

		setModuleStates(states);
	}

	public SwerveModulePosition[] getModulePositions() {
		return modules.stream().map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
	}

	private void initModulePIDF() {
		SmartDashboard.putNumber("kDriveP", Drive.kP);
		SmartDashboard.putNumber("kDriveI", Drive.kI);
		SmartDashboard.putNumber("kDriveD", Drive.kD);
		SmartDashboard.putNumber("kDriveFF", Drive.kFF);

		SmartDashboard.putNumber("kTurnP", Turn.kP);
		SmartDashboard.putNumber("kTurnI", Turn.kI);
		SmartDashboard.putNumber("kTurnD", Turn.kD);
		SmartDashboard.putNumber("kTurnFF", Turn.kFF);
	}
}
