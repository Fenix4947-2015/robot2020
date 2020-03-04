package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class Pigeon {

  private PigeonIMU _pigeon;

  private double _yaw;
  private double _pitch;
  private double _roll;
  private short _accelX;
  private short _accelY;
  private short _accelZ;
  private double _dpsX;
  private double _dpsY;
  private double _dpsZ;

  private double _zeroYaw = 0.0;
//  @Override
//  public void periodic() {
//    refresh();
//  }

  public void refresh() {
    double[] ypr = new double[3];
    _pigeon.getYawPitchRoll(ypr);
    _yaw = ypr[0];
    _pitch = ypr[1];
    _roll = ypr[2];

    short[] xyz = new short[3];
    _pigeon.getBiasedAccelerometer(xyz);
    _accelX = xyz[0];
    _accelY = xyz[1];
    _accelZ = xyz[2];

    double[] xyz_dps = new double[3];
    _pigeon.getRawGyro(xyz_dps);
    _dpsX = xyz_dps[0];
    _dpsY = xyz_dps[1];
    _dpsZ = xyz_dps[2];
  }

  public Pigeon(TalonSRX talon) {
    _pigeon = new PigeonIMU(talon);
    _pigeon.setTemperatureCompensationDisable(false);
    refresh();
  }

  public void resetHeading() {
    //_pigeon.setFusedHeading(0);
    _zeroYaw = getYaw();
  }

  public double getHeading() {
    //return _pigeon.getFusedHeading();
    return getYaw() - _zeroYaw;
  }

  public double getYaw() {
    return _yaw;
  }

  public double getPitch() {
    return _pitch;
  }

  public double getRoll() {
    return _roll;
  }

  public short getAccelX() {
    return _accelX;
  }

  public short getAccelY() {
    return _accelY;
  }

  public short getAccelZ() {
    return _accelZ;
  }

  public double getDpsX() {
    return _dpsX;
  }

  public double getDpsY() {
    return _dpsY;
  }

  public double getDpsZ() {
    return _dpsZ;
  }
}
