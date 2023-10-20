package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Swerve.Drift;

public class CorrectiveTeleop {
    public static PIDController driftController = new PIDController(Drift.kP, Drift.kI, Drift.kD);

    public static void initTunables(){
        SmartDashboard.putNumber("drift kp", SmartDashboard.getNumber("drift kp", Drift.kP));
        SmartDashboard.putNumber("drift ki", SmartDashboard.getNumber("drift ki", Drift.kI));
        SmartDashboard.putNumber("drift kd", SmartDashboard.getNumber("drift kd", Drift.kD));
    }

    public static void tuneController(){
        double kp = SmartDashboard.getNumber("drift kp", Drift.kP);
        double ki = SmartDashboard.getNumber("drift ki", Drift.kI);
        double kd = SmartDashboard.getNumber("drift kd", Drift.kD);

        driftController.setPID(kp, ki, kd);
    }

    // generate new throttle and strafe inputs to correct drift using component analysis and a feedback controller
    
    // inputThrottle should be the desired velocity in the forward direction of the gyro AS ORIENTED ON STARTUP
    // inputStrafe should be the desired velocity in the horizontal direction of the gyro AS ORIENTED ON STARTUP

    public static Translation2d generateCorrectedInput(double inputThrottle, double inputStrafe, AHRS gyro){
        // TODO: which component is this with our mounting?
        // velocity reading in the forward direction of the gyro AS CURRENTLY ORIENTED
        double robotThrottle = gyro.getVelocityY();
        SmartDashboard.putNumber("botThrottle", robotThrottle);

        // TODO: which component is this with our mounting?
        // velocity reading in the horizontal direction of the gyro AS CURRENTLY ORIENTED
        double robotStrafe = gyro.getVelocityX();
        SmartDashboard.putNumber("botStrafe", robotStrafe);

        // angle of the velocity reading with respect to the gyro yaw AS CURRENTLY ORIENTED
        double robotCentricRobotVelocityAngle = Math.atan2(robotThrottle, robotStrafe);
        SmartDashboard.putNumber("rcBotAngle", robotCentricRobotVelocityAngle * 180.0 / Math.PI);

        // angle of the velocity reading with respect to the gyro yaw AS ORIENTED ON STARTUP
        double fieldCentricRobotVelocityAngle = robotCentricRobotVelocityAngle + gyro.getYaw();
        SmartDashboard.putNumber("fcBotVelAngle", fieldCentricRobotVelocityAngle * 180.0 / Math.PI);

        // angle of the input velocity with respect to the gyro yaw AS ORIENTED ONSTARTUP
        double fieldCentricInputVelocityAngle = Math.atan2(inputThrottle, inputStrafe);
        SmartDashboard.putNumber("fcInputVelAngle", fieldCentricInputVelocityAngle * 180.0 / Math.PI);

        // difference between the angle of the velocity reading and the angle of the desired velocity (both with respect to the gyro yaw AS ORIENTED ON STARTUP)
        // important for this to be [ Error = Target - Measured ] so it has the same sign as the desired change in the angle of velocity
        double errorInVelocityAngle = fieldCentricInputVelocityAngle - fieldCentricRobotVelocityAngle;
        SmartDashboard.putNumber("errorVelAngle", errorInVelocityAngle * 180.0 / Math.PI);

        // magnitude of the component of the velocity reading that is perpendicular to the desired velocity
        double perpendicularErrorMagnitude = Math.sqrt(robotThrottle * robotThrottle + robotStrafe * robotStrafe) * Math.sin(errorInVelocityAngle);
        SmartDashboard.putNumber("perpErrorMag", perpendicularErrorMagnitude);

        // use a pid controller to generate the corrective output needed to minimize this error
        double perpendicularCorrectiveMagnitude = driftController.calculate(perpendicularErrorMagnitude, 0.0);
        SmartDashboard.putNumber("perpCorrectiveMag", perpendicularCorrectiveMagnitude);

        // TODO: potentially should be -pi/2
        // angle perpendicular to the input velocity with respect to the gyro yaw AS ORIENTED ON STARTUP
        double correctiveBasisVectorAngle = fieldCentricInputVelocityAngle + Math.PI / 2;
        SmartDashboard.putNumber("correctiveBasisVecAngle", correctiveBasisVectorAngle * 180.0 / Math.PI);

        // components to be added to input vector to correct drift
        double correctiveThrottle = perpendicularCorrectiveMagnitude * Math.sin(correctiveBasisVectorAngle);
        double correctiveStrafe = perpendicularCorrectiveMagnitude * Math.cos(correctiveBasisVectorAngle);
        SmartDashboard.putNumber("correctiveThrottle", correctiveThrottle);
        SmartDashboard.putNumber("correctiveStrafe", correctiveStrafe);

        // resultant components that preserve the desired velocity parallel to the input vector but correct the error in velocity perpendicular to the input vector
        double resultantThrottle = inputThrottle + correctiveThrottle;
        double resultantStrafe = inputStrafe + correctiveStrafe;
        SmartDashboard.putNumber("resultantThrottle", resultantThrottle);
        SmartDashboard.putNumber("resultantStrafe", resultantStrafe);

        return new Translation2d(resultantThrottle, resultantStrafe);
    }
}
