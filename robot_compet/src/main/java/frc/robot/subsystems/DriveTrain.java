package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveTrain extends SubsystemBase {

  // main motion system
  private final CANSparkMax leftMotor1 = new CANSparkMax(RobotMap.LEFT_MOTOR1_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax leftMotor2 = new CANSparkMax(RobotMap.LEFT_MOTOR2_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax rightMotor1 = new CANSparkMax(RobotMap.RIGHT_MOTOR1_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax rightMotor2 = new CANSparkMax(RobotMap.RIGHT_MOTOR2_CAN_ID, MotorType.kBrushless);
  private final DifferentialDrive robotDrive = new DifferentialDrive(leftMotor1, rightMotor1);

  private final Solenoid shifterSolenoid = RobotMap.SHIFTER_SOLENOID_CHANNEL_ID != null
      ? new Solenoid(RobotMap.SHIFTER_SOLENOID_CHANNEL_ID)
      : null;

  public DriveTrain() {
    // Initialize drivetrain motors
    // setMotorsAllowablePower(leftMotor1);
    // setMotorsAllowablePower(leftMotor2);

    // setMotorsAllowablePower(rightMotor1);
    // setMotorsAllowablePower(rightMotor2);

    // leftMotor2.setInverted(false);
    // leftMotor2.set(ControlMode.Follower, leftMotor1.getDeviceID());

    // rightMotor2.setInverted(false);
    // rightMotor2.set(ControlMode.Follower, rightMotor1.getDeviceID());

    // robotDrive.setSafetyEnabled(false);
    leftMotor1.restoreFactoryDefaults();
    leftMotor2.restoreFactoryDefaults();

    rightMotor1.restoreFactoryDefaults();
    rightMotor2.restoreFactoryDefaults();

    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);

    leftMotor1.setIdleMode(IdleMode.kBrake);
    leftMotor2.setIdleMode(IdleMode.kBrake);
    rightMotor1.setIdleMode(IdleMode.kBrake);
    rightMotor2.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {

  }

  public void driveArcadeMethod(double speed, double rotation) {

    double rotationValueGain = 1.0; // for full rotation speed, use 1. Tune to have smoother rotation.
    rotation = rotation * rotationValueGain;

    double goStraightCompensation = 0;
    if (Math.abs(speed) > 0.1) {
      // TODO Tune the constant values. Has a speed proportional component (friction
      // in mechanism() and a fixed component
      goStraightCompensation = speed * DriveTrainConstants.GO_STRAIGHT_COMPENSATION_DYNAMIC
          + DriveTrainConstants.GO_STRAIGHT_COMPENSATION_STATIC * Math.signum(speed);
    }

    robotDrive.arcadeDrive(speed, rotation + goStraightCompensation);
  }

  public void stop() {
    robotDrive.arcadeDrive(0.0, 0.0);
  }

  public void shiftHigh() {
    if (shifterSolenoid != null) {
      shifterSolenoid.set(true);
    }
  }

  public void shiftLow() {
    if (shifterSolenoid != null) {
      shifterSolenoid.set(false);
    }
  }

}
