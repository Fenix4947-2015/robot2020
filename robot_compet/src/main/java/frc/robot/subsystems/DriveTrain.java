package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.drivetrain.DriveArcade;

public class DriveTrain extends Subsystem {

  // main motion system
  private CANSparkMax leftMotor1 = new CANSparkMax(33, MotorType.kBrushless); 
  private CANSparkMax leftMotor2 = new CANSparkMax(34, MotorType.kBrushless);
  private CANSparkMax rightMotor1 = new CANSparkMax(21, MotorType.kBrushless); 
  private CANSparkMax rightMotor2 = new CANSparkMax(22, MotorType.kBrushless);
  private DifferentialDrive robotDrive = new DifferentialDrive(leftMotor1, rightMotor1);

  // Sensors
  private WPI_TalonSRX pigeonTalon = new WPI_TalonSRX(8);
  public Pigeon pigeon = new Pigeon();


  public DriveTrain() {
    // Initialize drivetrain motors
    //setMotorsAllowablePower(leftMotor1);
    //setMotorsAllowablePower(leftMotor2);

    //setMotorsAllowablePower(rightMotor1);
    //setMotorsAllowablePower(rightMotor2);

    //leftMotor2.setInverted(false);
    //leftMotor2.set(ControlMode.Follower, leftMotor1.getDeviceID());

    //rightMotor2.setInverted(false);
    //rightMotor2.set(ControlMode.Follower, rightMotor1.getDeviceID());

    //robotDrive.setSafetyEnabled(false);
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

  public void driveArcadeMethod(double Speed, double Rotation) {

    double rotationValueGain = 0.70; // for full rotation speed, use 1. Tune to have smoother rotation.
    Rotation = Rotation * rotationValueGain;

    double GoStraightCompensation = 0;
    if (Math.abs(Speed) > 0.1) {
      // TODO Tune the constant values. Has a speed proportional component (friction
      // in mechanism() and a fixed component
      GoStraightCompensation = Speed * DriveTrainConstants.GO_STRAIGHT_COMPENSATION_DYNAMIC
          + DriveTrainConstants.GO_STRAIGHT_COMPENSATION_STATIC * Math.signum(Speed);
    }

    robotDrive.arcadeDrive(Speed, Rotation + GoStraightCompensation);
  }

  public void Stop()
  {
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

    public Pigeon()
    {
        pigeon = new PigeonIMU(pigeonTalon);
        pigeon.setTemperatureCompensationDisable(false);
        refresh();
    }

    public void reset()
    {
      pigeon.setFusedHeading(0);      
    }

    public double getHeading()
    {
      return pigeon.getFusedHeading();
    }

  }

  public void log()
  {
    SmartDashboard.putNumber("banane", 3.14169);
  }

}
