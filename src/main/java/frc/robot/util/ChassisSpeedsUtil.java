package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisSpeedsUtil {
    // currently not in wpilib, is set for 2024 release?
    public static ChassisSpeeds discretize(
        double vxMetersPerSecond,
        double vyMetersPerSecond,
        double omegaRadiansPerSecond,
        double dtSeconds
    ) {
        var desiredDeltaPose = new Pose2d(
            vxMetersPerSecond * dtSeconds,
                vyMetersPerSecond * dtSeconds,
                new Rotation2d(omegaRadiansPerSecond * dtSeconds)
        );

        var twist = new Pose2d().log(desiredDeltaPose);
        return new ChassisSpeeds(
            twist.dx / dtSeconds,
            twist.dy / dtSeconds,
            twist.dtheta / dtSeconds
        );
    }

    public static ChassisSpeeds discretize(ChassisSpeeds speeds, double dtSeconds) {
        return discretize(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            dtSeconds
        );
    }
}