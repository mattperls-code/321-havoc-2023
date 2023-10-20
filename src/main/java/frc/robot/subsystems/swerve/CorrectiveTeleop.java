package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CorrectiveTeleop {
    public static Translation2d generateCorrectedInput(double inputThrottle, double inputStrafe, double inputOmega){
        double inputSpeed = Math.sqrt(inputThrottle * inputThrottle + inputStrafe * inputStrafe);

        if (inputSpeed == 0 || inputOmega == 0) return new Translation2d(inputThrottle, inputStrafe);

        // calculate the angle of input strafe, rotate 90 degrees in opposite direction of omega
        double inputStrafeAngle = Math.atan2(inputThrottle, inputStrafe);

        // TODO: plus or minus 90 degrees?
        double correctiveStrafeAngle = inputStrafeAngle + 0.5 * Math.PI * Math.signum(inputOmega);

        double kP = SmartDashboard.getNumber("corrective kp", 0.0);

        // corrective strafe speed should be proportional to both the desired strafe speed as well as the desired angular speed
        double correctiveSpeed = kP * inputSpeed * Math.abs(inputOmega);
        double correctiveThrottle = correctiveSpeed * Math.cos(correctiveStrafeAngle);
        double correctiveStrafe = correctiveSpeed * Math.sin(correctiveStrafeAngle);

        // add corrective strafe to input
        double correctedThrottle = inputThrottle + correctiveThrottle;
        double correctedStrafe = inputStrafe + correctiveStrafe;

        return new Translation2d(correctedThrottle, correctedStrafe);
    }
}
