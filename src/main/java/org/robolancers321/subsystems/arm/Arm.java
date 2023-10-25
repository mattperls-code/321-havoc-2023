/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.invoke.ConstantCallSite;

import org.robolancers321.Constants;

public class Arm extends SubsystemBase {
  private final CANSparkMax anchorMotor;
  private final CANSparkMax floatingMotor;

  private final AbsoluteEncoder anchorEncoder;
  private final AbsoluteEncoder floatingEncoder;

  private final SparkMaxPIDController anchorPIDController;
  private final SparkMaxPIDController floatingPIDController;

  public double anchorSetpoint;
  public double floatingSetpoint;
  public TrapezoidProfile anchorProfile = new TrapezoidProfile(Constants.Arm.Anchor.ANCHOR_CONSTRAINTS, new TrapezoidProfile.State());
  public TrapezoidProfile floatingProfile = new TrapezoidProfile(Constants.Arm.Floating.FLOATING_CONSTRAINTS, new TrapezoidProfile.State());
  public double profileStartTime;
  public double kG = 0.46;

  public Arm() {
    this.anchorMotor = new CANSparkMax(Constants.Arm.Anchor.kAnchorPort, MotorType.kBrushless);
    this.anchorEncoder = anchorMotor.getAbsoluteEncoder(Type.kDutyCycle);
    this.anchorPIDController = this.anchorMotor.getPIDController();

    this.floatingMotor =
        new CANSparkMax(Constants.Arm.Floating.kFloatingPort, MotorType.kBrushless);
    this.floatingEncoder = floatingMotor.getAbsoluteEncoder(Type.kDutyCycle);
    this.floatingPIDController = this.floatingMotor.getPIDController();

    configureMotors();
    configureEncoders();
    configureControllers();

    this.anchorSetpoint = getAnchorAngle();
    this.floatingSetpoint = getFloatingAngle();
    initTuneControllers();
  }

  private void configureMotors() {
    anchorMotor.setInverted(Constants.Arm.Anchor.kMotorInverted);
    anchorMotor.setIdleMode(IdleMode.kBrake);
    anchorMotor.setSmartCurrentLimit(Constants.Arm.Anchor.kCurrentLimit);
    anchorMotor.enableVoltageCompensation(Constants.Arm.Anchor.kNominalVoltage);
    anchorMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Arm.Anchor.kMinAngle);
    anchorMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Arm.Anchor.kMaxAngle);
    anchorMotor.enableSoftLimit(SoftLimitDirection.kReverse, Constants.Arm.Anchor.kEnableSoftLimit);
    anchorMotor.enableSoftLimit(SoftLimitDirection.kForward, Constants.Arm.Anchor.kEnableSoftLimit);

    // //limit feedback rate of useless info, give abs encoder feedback at 20 m/s. 65535 m/s is the
    // max
    anchorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // analog sensor
    anchorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // alternate encoder
    anchorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); // absolute encoder
    anchorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); // absolute encoder

    floatingMotor.setInverted(Constants.Arm.Floating.kMotorInverted);
    floatingMotor.setIdleMode(IdleMode.kBrake);
    floatingMotor.setSmartCurrentLimit(Constants.Arm.Floating.kCurrentLimit);
    floatingMotor.enableVoltageCompensation(Constants.Arm.Floating.kNominalVoltage);
    floatingMotor.setSoftLimit(
        SoftLimitDirection.kReverse, (float) Constants.Arm.Floating.kMinAngle);
    floatingMotor.setSoftLimit(
        SoftLimitDirection.kForward, (float) Constants.Arm.Floating.kMaxAngle);
    floatingMotor.enableSoftLimit(
        SoftLimitDirection.kReverse, Constants.Arm.Floating.kEnableSoftLimit);
    floatingMotor.enableSoftLimit(
        SoftLimitDirection.kForward, Constants.Arm.Floating.kEnableSoftLimit);

    floatingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    floatingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    floatingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    floatingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
  }

  private void configureEncoders() {
    anchorEncoder.setPositionConversionFactor(Constants.Arm.Anchor.kdistancePerRotation);
    anchorEncoder.setVelocityConversionFactor(Constants.Arm.Anchor.kdistancePerRotation);
    anchorEncoder.setInverted(Constants.Arm.Anchor.kEncoderInverted);
    anchorEncoder.setZeroOffset(Constants.Arm.Anchor.kZeroPosition);

    floatingEncoder.setPositionConversionFactor(Constants.Arm.Floating.kdistancePerRotation);
    floatingEncoder.setVelocityConversionFactor(Constants.Arm.Floating.kdistancePerRotation);
    floatingEncoder.setInverted(Constants.Arm.Floating.kEncoderInverted);
    floatingEncoder.setZeroOffset(Constants.Arm.Floating.kZeroPosition);
    // floatingEncoder.setPosition(Constants.Arm.Floating.kZeroPosition);
  }

  private void configureControllers() {
    anchorPIDController.setP(Constants.Arm.Anchor.kP);
    anchorPIDController.setI(Constants.Arm.Anchor.kI);
    anchorPIDController.setD(Constants.Arm.Anchor.kD);
    anchorPIDController.setOutputRange(
        Constants.Arm.Anchor.kMinOutput, Constants.Arm.Anchor.kMaxOutput);
    anchorPIDController.setPositionPIDWrappingEnabled(true);
    anchorPIDController.setPositionPIDWrappingMinInput(0.0);
    anchorPIDController.setPositionPIDWrappingMaxInput(360.0);

    floatingPIDController.setP(Constants.Arm.Floating.kP);
    floatingPIDController.setI(Constants.Arm.Floating.kI);
    floatingPIDController.setD(Constants.Arm.Floating.kD);
    floatingPIDController.setOutputRange(
        Constants.Arm.Floating.kMinOutput, Constants.Arm.Floating.kMaxOutput);
    floatingPIDController.setPositionPIDWrappingEnabled(true);
    floatingPIDController.setPositionPIDWrappingMinInput(0.0);
    floatingPIDController.setPositionPIDWrappingMaxInput(359.0);

  }

  public double getAnchorAngle() {
    return anchorEncoder.getPosition();
  }

  public double getFloatingAngle() {
    return floatingEncoder.getPosition();
  }


  public double getAnchorVelocity() {
    return anchorEncoder.getVelocity();
  }

  public double getFloatingVelocity() {
    return floatingEncoder.getVelocity();
  }

  public void setAnchorSpeed(double speed) {
    anchorPIDController.setReference(speed, ControlType.kDutyCycle);
  }

  public void setFloatingSpeed(double speed) {
    anchorPIDController.setReference(speed, ControlType.kDutyCycle);
  }

  public void setAnchorControllerReference(double reference, double ff) {
    anchorPIDController.setReference(
        reference, ControlType.kPosition, Constants.Arm.Anchor.kPIDSlot, ff, ArbFFUnits.kVoltage);
  }

  public void setFloatingControllerReference(double reference, double ff) {
    floatingPIDController.setReference(
        reference, ControlType.kPosition, Constants.Arm.Floating.kPIDSlot, ff, ArbFFUnits.kVoltage);
  }

  public double getAnchorSetpoint() {
    return this.anchorSetpoint;
  }

  public double getFloatingSetpoint() {
    return this.floatingSetpoint;
  }

  public void setAnchorSetpoint(double position) {
    this.anchorSetpoint = position;
  }

  public void setFloatingSetpoint(double position) {
    this.floatingSetpoint = position;
  }

  public double calculateAnchorFF(){
    double alpha = Math.toRadians(getAnchorAngle());
    double beta = Math.toRadians(getFloatingAngle()); //relative to horizontal
    double l1 = Constants.Arm.Anchor.kAnchorLength; 
    double l2 = Constants.Arm.Floating.kFloatingLength;

    double anchorFF = this.kG * (l1 * Math.cos(alpha) + l2 * Math.cos(beta));

    return getAnchorAngle() > 90 ? -anchorFF : anchorFF;
  }

  public double calculateFloatingFF(){
    double beta = Math.toRadians(getFloatingAngle()); //relative to horizontal
    double l2 = Constants.Arm.Floating.kFloatingLength;

    double floatingFF = this.kG * (l2 * Math.cos(beta));
    return floatingFF;
  }

  

  private void initTuneControllers() {
    SmartDashboard.putNumber(
        "anchorSetpoint", SmartDashboard.getNumber("anchorSetpoint", this.anchorSetpoint));
    SmartDashboard.putNumber(
        "floatingSetpoint", SmartDashboard.getNumber("floatingSetpoint", this.floatingSetpoint));
    SmartDashboard.putNumber(
        "anchorMaxVEL", SmartDashboard.getNumber("anchorMaxVEL", Constants.Arm.Anchor.maxVelocity));
    SmartDashboard.putNumber(
        "anchorMaxACCEL", SmartDashboard.getNumber("anchorMaxACCEL", Constants.Arm.Anchor.maxAcceleration));
    SmartDashboard.putNumber(
        "floatingMaxVEL", SmartDashboard.getNumber("floatingMaxVEL", Constants.Arm.Floating.maxVelocity));
    SmartDashboard.putNumber(
        "floatingMaxACCEL", SmartDashboard.getNumber("floatingMaxACCEL", Constants.Arm.Floating.maxAcceleration));

    SmartDashboard.putNumber(
        "anchorKP", SmartDashboard.getNumber("anchorKP", Constants.Arm.Anchor.kP));
    SmartDashboard.putNumber(
        "anchorKI", SmartDashboard.getNumber("anchorKI", Constants.Arm.Anchor.kI));
    SmartDashboard.putNumber(
        "anchorKD", SmartDashboard.getNumber("anchorKD", Constants.Arm.Anchor.kD));

    SmartDashboard.putNumber(
        "floatingKP", SmartDashboard.getNumber("floatingKP", Constants.Arm.Floating.kP));
    SmartDashboard.putNumber(
        "floatingKI", SmartDashboard.getNumber("floatingKI", Constants.Arm.Floating.kI));
    SmartDashboard.putNumber(
        "floatingKD", SmartDashboard.getNumber("floatingKD", Constants.Arm.Floating.kD));
    SmartDashboard.putNumber(
        "customKG", SmartDashboard.getNumber("customKG", this.kG));
  }

  private void tuneControllers() {
    double anchorSetpoint = SmartDashboard.getEntry("anchorSetpoint").getDouble(0);
    double anchorKP = SmartDashboard.getEntry("anchorKP").getDouble(0);
    double anchorKI = SmartDashboard.getEntry("anchorKI").getDouble(0);
    double anchorKD = SmartDashboard.getEntry("anchorKD").getDouble(0);
    double customKG = SmartDashboard.getEntry("customKG").getDouble(0);
    double anchorMaxVEL = SmartDashboard.getEntry("anchorMaxVEL").getDouble(0);
    double anchorMaxACCEL = SmartDashboard.getEntry("anchorMaxACCEL").getDouble(0);

    double floatingSetpoint = SmartDashboard.getEntry("floatingSetpoint").getDouble(0);
    double floatingKP = SmartDashboard.getEntry("floatingKP").getDouble(0);
    double floatingKI = SmartDashboard.getEntry("floatingKI").getDouble(0);
    double floatingKD = SmartDashboard.getEntry("floatingKD").getDouble(0);
    double floatingMaxVel = SmartDashboard.getEntry("floatingMaxVEL").getDouble(0);
    double floatingMaxAccel = SmartDashboard.getEntry("floatingMaxACCEL").getDouble(0);
    
    Constants.Arm.Anchor.ANCHOR_CONSTRAINTS = new TrapezoidProfile.Constraints(anchorMaxVEL, anchorMaxACCEL);
    Constants.Arm.Floating.FLOATING_CONSTRAINTS = new TrapezoidProfile.Constraints(floatingMaxVel, floatingMaxAccel);

    this.anchorPIDController.setP(anchorKP);
    this.anchorPIDController.setI(anchorKI);
    this.anchorPIDController.setD(anchorKD);
    this.anchorSetpoint =
        MathUtil.clamp(
            anchorSetpoint, Constants.Arm.Anchor.kMinAngle, Constants.Arm.Anchor.kMaxAngle);
    this.kG = customKG;

    this.floatingPIDController.setP(floatingKP);
    this.floatingPIDController.setI(floatingKI);
    this.floatingPIDController.setD(floatingKD);
    this.floatingSetpoint =
        MathUtil.clamp(
            floatingSetpoint, Constants.Arm.Floating.kMinAngle, Constants.Arm.Floating.kMaxAngle);

    SmartDashboard.putNumber("anchorAngle", getAnchorAngle());
    SmartDashboard.putNumber("floatingAngle", getFloatingAngle());
    
    SmartDashboard.putNumber("anchorOutput", this.anchorMotor.getAppliedOutput());
    SmartDashboard.putNumber("floatingOutput", this.floatingMotor.getAppliedOutput());
  }

  @Override
  public void periodic() {

    // double z = SmartDashboard.getEntry("z").getDouble(0);
    // double y = SmartDashboard.getEntry("y").getDouble(0);

    // InverseArmKinematics.Output angles = InverseArmKinematics.calculate(y - 17, z+12);

    // SmartDashboard.putNumber("anchorAngleIK", angles.anchor);
    // SmartDashboard.putNumber("floatingAngleIK", angles.anchor - angles.floating);

    tuneControllers();
  }
}
