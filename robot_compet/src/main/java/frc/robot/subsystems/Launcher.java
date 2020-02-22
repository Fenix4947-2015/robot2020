/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Objects;

import java.time.Instant;
import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FileLogger;
import frc.robot.RobotMap;
import frc.robot.SmartDashboardSettings;
import frc.robot.commands.launcher.KeepBallInRamp;

/**
 * Add your docs here.
 */
public class Launcher extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public final SmartDashboardSettings _smartDashboardSettings;

  private final CANSparkMax motorWheelUp;
  private final CANPIDController pidWheelUp;
  private final CANSparkMax motorWheelDown;
  private final CANPIDController pidWheelDown;
  private final Solenoid ramp;

  private static final double DOWN_WHEEL_SPEED = 0.8;
  private static final double PRE_SPIN_DOWN_WHEEL_SPEED = 0.8; // 0.7;
  private static final double UP_WHEEL_TO_DOWN_WHEEL_SPEED_RATIO = 0.1875;

  private static final double TARGET_SPEED_DOWN = 4200;
  private static final double TARGET_SPEED_UP = 750;
  private static final double TOLERANCE_SPEED = 10;

  private FileLogger fileLogger;
  private Instant startTime = Instant.now();
  private String phase = "unknown";

  public Launcher(SmartDashboardSettings smartDashboardSettings) {
    _smartDashboardSettings = smartDashboardSettings;

    System.out.println("Launcher constructor");
    setDefaultCommand(new KeepBallInRamp(this));

    motorWheelUp = new CANSparkMax(RobotMap.LAUNCHER_MOTOR_UP_CAN_ID, MotorType.kBrushless);
    motorWheelUp.setInverted(false);
    motorWheelUp.enableVoltageCompensation(12.0);
    pidWheelUp = new CANPIDController(motorWheelUp);
    pidWheelUp.setP(5e-6);
    pidWheelUp.setI(0.00e-8);
    pidWheelUp.setD(0);
    pidWheelUp.setIZone(0);
    pidWheelUp.setFF(0.001/4300.0*TARGET_SPEED_UP);
    pidWheelUp.setOutputRange(-5700, 5700);
    pidWheelUp.setFF(0.0);

    motorWheelDown = new CANSparkMax(RobotMap.LAUNCHER_MOTOR_DOWN_CAN_ID, MotorType.kBrushless);
    motorWheelDown.setInverted(true);
    motorWheelDown.enableVoltageCompensation(12.0);
    pidWheelDown = new CANPIDController(motorWheelDown);
    pidWheelDown.setP(1.2e-4);
    pidWheelDown.setI(0.0);
    pidWheelDown.setD(0);
    pidWheelDown.setIZone(0);
    //pidWheelDown.setFF(0.0001/2300.0*TARGET_SPEED_DOWN);
    pidWheelDown.setFF(0.0);

    pidWheelDown.setOutputRange(-5700, 5700);

    ramp = new Solenoid(RobotMap.RAMP_SOLENOID_CHANNEL_ID);
  }

  @Override
  public void periodic() {
    _smartDashboardSettings.refreshPidValues();
    if (Objects.equals(_smartDashboardSettings.getPidType(), "LAUNCHERUP")) {
      setPidWheelUp(_smartDashboardSettings.getPidP(), _smartDashboardSettings.getPidI(),
          _smartDashboardSettings.getPidD(), _smartDashboardSettings.getPidF());
    }
    if (Objects.equals(_smartDashboardSettings.getPidType(), "LAUNCHERDOWN")) {
      setPidWheelDown(_smartDashboardSettings.getPidP(), _smartDashboardSettings.getPidI(),
          _smartDashboardSettings.getPidD(), _smartDashboardSettings.getPidF());
    }
  }

  public void setPidWheelUp(double p, double i, double d, double f){
    pidWheelUp.setP(p);
    pidWheelUp.setI(i);
    pidWheelUp.setD(d);
    //pidWheelUp.setFF(f);
  }

  public void setPidWheelDown(double p, double i, double d, double f){
    System.out.println(String.format("pid: %f %f %f %f", p, i, d, f));
    pidWheelDown.setP(p);
    pidWheelDown.setI(i);
    pidWheelDown.setD(d);
    //pidWheelDown.setFF(f);
  }

  public void shootPIDRPM() {
    pidWheelDown.setReference(TARGET_SPEED_DOWN, ControlType.kVelocity);
    pidWheelUp.setReference(TARGET_SPEED_UP, ControlType.kVelocity); 
    logSpeed();
  }

  public boolean isAtTargetSpeed() {
    double currentDownSpeed = motorWheelDown.getEncoder().getVelocity();
    double currentUpSpeed = motorWheelUp.getEncoder().getVelocity();
    double totalError = Math.abs(currentDownSpeed - TARGET_SPEED_DOWN) + Math.abs(currentUpSpeed - TARGET_SPEED_UP);
    return (totalError <= TOLERANCE_SPEED);
  }

  private void logSpeed() {
    long msSincePhaseStart = Instant.now().toEpochMilli() - startTime.toEpochMilli();
    double upVelocity = motorWheelUp.getEncoder().getVelocity();
    double downVelocity = motorWheelDown.getEncoder().getVelocity();
    fileLogger.writeText(String.format("%s,%d,%f,%f%n", phase, msSincePhaseStart, upVelocity, downVelocity));
  }

  private void initLogging() {
    startTime = Instant.now();
    stopLogging();
    fileLogger = new FileLogger(String.format("launcher_%s_%s.csv",
        DateTimeFormatter.ofPattern("yyyyMMddHHmmss").format(ZonedDateTime.now()), phase), true);
  }

  public void setCurrentPhase(String phase) {
    this.phase = phase;
    initLogging();
  }

  public void openLoopShoot(boolean isPreSpin) {
    final double downWheelSpeed = isPreSpin ? PRE_SPIN_DOWN_WHEEL_SPEED : DOWN_WHEEL_SPEED;
    final double upWheelSpeed = downWheelSpeed * UP_WHEEL_TO_DOWN_WHEEL_SPEED_RATIO;

    motorWheelUp.set(upWheelSpeed);
    motorWheelDown.set(downWheelSpeed);

  }



  public void stop() {
    motorWheelUp.set(0.0);
    motorWheelDown.set(0.0);
    stopLogging();
  }

  private void stopLogging() {
    if (fileLogger != null) {
      fileLogger.close();
      fileLogger = null;
    }
  }

  public void done() {
    stopLogging();
  }

  public void rampUp() {
    ramp.set(true);
  }

  public void rampDown() {
    ramp.set(false);
  }

  public void log() {
    SmartDashboard.putNumber("Encoder Speed Up", motorWheelUp.getEncoder().getVelocity());
    SmartDashboard.putNumber("Encoder Speed Down", motorWheelDown.getEncoder().getVelocity());
  }

}
