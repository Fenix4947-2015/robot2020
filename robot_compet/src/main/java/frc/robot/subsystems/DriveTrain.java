package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.drivetrain.DriveArcade;

public class DriveTrain extends Subsystem {

  // main motion system
  private CANSparkMax leftMotor1 = new CANSparkMax(RobotMap.LEFT_MOTOR1_CAN_ID, MotorType.kBrushless);
  private CANSparkMax leftMotor2 = new CANSparkMax(RobotMap.LEFT_MOTOR2_CAN_ID, MotorType.kBrushless);
  private CANSparkMax rightMotor1 = new CANSparkMax(RobotMap.RIGHT_MOTOR1_CAN_ID, MotorType.kBrushless);
  private CANSparkMax rightMotor2 = new CANSparkMax(RobotMap.RIGHT_MOTOR2_CAN_ID, MotorType.kBrushless);
  private DifferentialDrive robotDrive = new DifferentialDrive(leftMotor1, rightMotor1);

  // Sensors
  //private WPI_TalonSRX pigeonTalon = new WPI_TalonSRX(8);
  //public Pigeon pigeon = new Pigeon(pigeonTalon);

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

  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveArcade());
  }

  public void driveArcadeMethod(double speed, double rotation) {

    double rotationValueGain = 0.70; // for full rotation speed, use 1. Tune to have smoother rotation.
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

  public class Pigeon {

    private PigeonIMU pigeon;

    public double yaw;
    public double pitch;
    public double roll;
    public short accelX;
    public short accelY;
    public short accelZ;

    public void refresh() {
      double[] ypr = new double[3];
      pigeon.getYawPitchRoll(ypr);
      yaw = ypr[0];
      pitch = ypr[1];
      roll = ypr[2];

      short[] xyz = new short[3];
      pigeon.getBiasedAccelerometer(xyz);
      accelX = xyz[0];
      accelY = xyz[1];
      accelZ = xyz[2];
    }

    public Pigeon(TalonSRX talon) {
      pigeon = new PigeonIMU(talon);
      pigeon.setTemperatureCompensationDisable(false);
      refresh();
    }

    public void reset() {
      pigeon.setFusedHeading(0);
    }

    public double getHeading() {
      return pigeon.getFusedHeading();
    }

  }

  public void log() {
    SmartDashboard.putNumber("banane", 3.14169);
  }



}
