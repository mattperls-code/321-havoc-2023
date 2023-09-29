package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;

public class MoveToSetpoint extends CommandBase{

    private double anchorPosSetpoint;
    private double anchorVelSetpoint;
    private double floatingPosSetpoint;
    private double floatingVelSetpoint;
    Arm arm;

    public MoveToSetpoint(Arm arm, Constants.Arm.Anchor.Setpoints anchorSetpoint, Constants.Arm.Floating.Setpoints floatingSetpoint){
        this.anchorPosSetpoint = anchorSetpoint.position;
        this.anchorVelSetpoint = anchorSetpoint.velocity;
        this.floatingPosSetpoint = floatingSetpoint.position;
        this.floatingVelSetpoint = floatingSetpoint.position;
        this.arm = arm;

        addRequirements(arm);
    }

    @Override
    public void initialize(){
        arm.periodicIO.anchorAngle = anchorPosSetpoint;
        arm.periodicIO.anchorVelocity = anchorVelSetpoint;
        arm.periodicIO.floatingAngle = floatingPosSetpoint;
        arm.periodicIO.floatingVelocity = floatingVelSetpoint;
    }

    @Override
    public boolean isFinished() {
        return arm.getAnchorVelocity() == anchorVelSetpoint 
        && arm.getAnchorAngle() == anchorPosSetpoint
        && arm.getFloatingVelocity() == floatingVelSetpoint
        && arm.getFloatingAngle() == floatingPosSetpoint;
    }

    @Override
    public void end(boolean interrupted){
        
    }
}
