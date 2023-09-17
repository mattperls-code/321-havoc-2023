package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    public static final double kPeriodSeconds = 0.02;
    public static class Swerve {
        // TODO; find these values
        public static final double kTrackWidthMeters = 0.0;
        public static final double kWheelBaseMeters = 0.0;

        public static final double kWheelRadiusMeters = 0.0381;
        public static final double kGearRatio = 6.8;

        public static final double kMaxSpeedMetersPerSecond = 0.5;

        public static final double kVelocityConversionFactor = 2 * Math.PI * kWheelRadiusMeters / (kGearRatio * 60.0);

        // TODO: set ids
        public static final int kFrontLeftDriveId = 0;
        public static final int kFrontRightDriveId = 0;
        public static final int kBackLeftDriveId = 0;
        public static final int kBackRightDriveId = 0;
        public static final int kFrontLeftTurnId = 0;
        public static final int kFrontRightTurnId = 0;
        public static final int kBackLeftTurnId = 0;
        public static final int kBackRightTurnId = 0;
        public static final int kFrontLeftTurnEncoderId = 0;
        public static final int kFrontRightTurnEncoderId = 0;
        public static final int kBackLeftTurnEncoderId = 0;
        public static final int kBackRightTurnEncoderId = 0;
        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
        new Translation2d(kTrackWidthMeters / 2, kWheelBaseMeters / 2), // front left
            new Translation2d(kTrackWidthMeters / 2, -kWheelBaseMeters / 2), // front right
            new Translation2d(-kTrackWidthMeters / 2, kWheelBaseMeters / 2), // back left
            new Translation2d(-kTrackWidthMeters / 2, -kWheelBaseMeters / 2) // back right
        );

        // TODO: tune coeffs.
        public static final double kDriveP = 0.0;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kTurnP = 0.0;
        public static final double kTurnI = 0.0;
        public static final double kTurnD = 0.0;
    }
}