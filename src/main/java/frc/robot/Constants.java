package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final double kPeriodSeconds = Robot.kDefaultPeriod;
    public static class Swerve {
        public static final class Module {
            public final int kDriveId;
            public final int kTurnId;
            public final int kTurnEncoderId;
            
            public Module(final int driveId, final int turnId, final int turnEncoderId) {
                this.kDriveId = driveId;
                this.kTurnId = turnId;
                this.kTurnEncoderId = turnEncoderId;
            }
        }
        
        public static final int kFrontLeftDriveId = 2;
        public static final int kFrontRightDriveId = 4;
        public static final int kBackLeftDriveId = 7;
        public static final int kBackRightDriveId = 6;
        public static final int kFrontLeftTurnId = 9;
        public static final int kFrontRightTurnId = 3;
        public static final int kBackLeftTurnId = 8;
        public static final int kBackRightTurnId = 5;
        public static final int kFrontLeftTurnEncoderId = 10;
        public static final int kFrontRightTurnEncoderId = 11;
        public static final int kBackLeftTurnEncoderId = 13;
        public static final int kBackRightTurnEncoderId = 12;

        public static final Module frontLeft = new Module(2, 9, 10);
        public static final Module frontRight = new Module(4, 3, 11);
        public static final Module backLeft = new Module(7, 8, 13);
        public static final Module backRight = new Module(6, 5, 12);

        public static final CANCoderConfiguration kCANCoderConfig = new CANCoderConfiguration();

        static {
            kCANCoderConfig.sensorCoefficient = 2 * Math.PI / 4096.0;
            kCANCoderConfig.unitString = "rad";
            kCANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
            kCANCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        }

        public static final double kTrackWidthMeters = Units.inchesToMeters(17.5);
        public static final double kWheelBaseMeters = Units.inchesToMeters(17.5);

        public static final double kWheelRadiusMeters = Units.inchesToMeters(1.5);
        public static final double kGearRatio = 6.8;

        public static final double kMaxSpeedMetersPerSecond = 0.75;

        public static final double kRPMToMetersPerSecond = 2 * Math.PI * kWheelRadiusMeters / (kGearRatio * 60.0);

        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
            new Translation2d(kTrackWidthMeters / 2, kWheelBaseMeters / 2), // front left
            new Translation2d(kTrackWidthMeters / 2, -kWheelBaseMeters / 2), // front right
            new Translation2d(-kTrackWidthMeters / 2, kWheelBaseMeters / 2), // back left
            new Translation2d(-kTrackWidthMeters / 2, -kWheelBaseMeters / 2) // back right
        );

        // TODO: tune coeffs.
        public static final class Drive {
            public static final double kP = 0.001;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kFF = 0.29;
        }
        
        public static final class Turn {
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kFF = 0.0;
        }
    }
}