package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pigeon extends SubsystemBase {

  private PigeonIMU _pigeon;

  private double _yaw;
  private double _pitch;
  private double _roll;
  private short _accelX;
  private short _accelY;
  private short _accelZ;

  @Override
  public void periodic() {
    refresh();
  }

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
  }

  public Pigeon(TalonSRX talon) {
    _pigeon = new PigeonIMU(talon);
    _pigeon.setTemperatureCompensationDisable(false);
    refresh();
  }

  public void reset() {
    _pigeon.setFusedHeading(0);
  }

  public double getHeading() {
    return _pigeon.getFusedHeading();
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
}
