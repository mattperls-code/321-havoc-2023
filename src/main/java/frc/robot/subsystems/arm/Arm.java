package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    public CANSparkMax anchorMotor;
    public CANSparkMax floatingMotor;

    public AbsoluteEncoder anchorEncoder;
    public AbsoluteEncoder floatingEncoder;

    public SparkMaxPIDController anchorPIDController;
    public SparkMaxPIDController floatingPIDController;

    public Arm(){
        this.anchorMotor = new CANSparkMax(Constants.Arm.Anchor.kAnchorPort, MotorType.kBrushless);
        this.anchorEncoder = anchorMotor.getAbsoluteEncoder(Type.kDutyCycle);
        this.anchorPIDController = this.anchorMotor.getPIDController(); 

        this.floatingMotor = new CANSparkMax(Constants.Arm.Floating.kFloatingPort, MotorType.kBrushless);
        this.floatingEncoder = floatingMotor.getAbsoluteEncoder(Type.kDutyCycle);
        this.floatingPIDController = this.floatingMotor.getPIDController(); 

        configureMotors();
        configureEncoders();
        configureControllers();
    }

    public void configureMotors(){
        anchorMotor.setInverted(Constants.Arm.Anchor.kInverted);
        anchorMotor.setIdleMode(IdleMode.kBrake);
        anchorMotor.setSmartCurrentLimit(Constants.Arm.Anchor.kCurrentLimit);
        //if battery is over 12 V then clamp motors at 12 V
        anchorMotor.enableVoltageCompensation(12.0);
        anchorMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Arm.Anchor.kMinAngle); 
        anchorMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Arm.Anchor.kMaxAngle);
        anchorMotor.enableSoftLimit(SoftLimitDirection.kReverse, Constants.Arm.Anchor.kEnableSoftLimit);
        anchorMotor.enableSoftLimit(SoftLimitDirection.kForward, Constants.Arm.Anchor.kEnableSoftLimit);

        floatingMotor.setInverted(Constants.Arm.Floating.kInverted);
        floatingMotor.setIdleMode(IdleMode.kBrake);
        floatingMotor.setSmartCurrentLimit(Constants.Arm.Floating.kCurrentLimit);
        floatingMotor.enableVoltageCompensation(12.0);
        floatingMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Arm.Floating.kMinAngle); 
        floatingMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Arm.Floating.kMaxAngle);
        floatingMotor.enableSoftLimit(SoftLimitDirection.kReverse, Constants.Arm.Floating.kEnableSoftLimit);
        floatingMotor.enableSoftLimit(SoftLimitDirection.kForward, Constants.Arm.Floating.kEnableSoftLimit);
    }

    public void configureEncoders(){
        anchorEncoder.setPositionConversionFactor(Constants.Arm.Anchor.kGearRatio);
        anchorEncoder.setVelocityConversionFactor(Constants.Arm.Anchor.kdistancePerRotation);
        anchorEncoder.setZeroOffset(Constants.Arm.Anchor.kZeroPosition);  

        floatingEncoder.setPositionConversionFactor(Constants.Arm.Floating.kGearRatio);
        floatingEncoder.setVelocityConversionFactor(Constants.Arm.Floating.kdistancePerRotation);
        floatingEncoder.setZeroOffset(Constants.Arm.Floating.kZeroPosition);  
    }

    public void configureControllers(){
        anchorPIDController.setP(Constants.Arm.Anchor.kP);
        anchorPIDController.setI(Constants.Arm.Anchor.kI);
        anchorPIDController.setD(Constants.Arm.Anchor.kD);
        anchorPIDController.setOutputRange(Constants.Arm.Anchor.kMinOutput, Constants.Arm.Anchor.kMaxOutput);

        floatingPIDController.setP(Constants.Arm.Floating.kP);
        floatingPIDController.setI(Constants.Arm.Floating.kI);
        floatingPIDController.setD(Constants.Arm.Floating.kD);
        floatingPIDController.setOutputRange(Constants.Arm.Floating.kMinOutput, Constants.Arm.Floating.kMaxOutput);
    }
}
