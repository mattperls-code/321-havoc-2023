package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardUtil {
    private SmartDashboardUtil() {
        // utility class
    }

    public static double pollOrDefault(String key, double defaultValue) {
        SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
        return SmartDashboard.getNumber(key, defaultValue);
    }
}