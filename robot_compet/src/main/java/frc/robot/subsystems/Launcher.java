/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.launcher.KeepBallInRamp;

/**
 * Add your docs here.
 */
public class Launcher extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private CANSparkMax motorWheelUp;
  private CANPIDController pidWheelUp;
  private CANSparkMax motorWheelDown;
  private CANPIDController pidWheelDown;
  private Solenoid ramp;

  private static final double DOWN_WHEEL_SPEED = 0.8;
  private static final double PRE_SPIN_DOWN_WHEEL_SPEED = 0.7;
  private static final double UP_WHEEL_TO_DOWN_WHEEL_SPEED_RATIO = 0.1875;

  private static final double TARGET_SPEED_DOWN = 900;
  private static final double TARGET_SPEED_UP = 5100;
  private static final double TOLERANCE_SPEED = 10;

  public Launcher() {
    System.out.println("Launcher constructor");

    motorWheelUp = new CANSparkMax(RobotMap.LAUNCHER_MOTOR_UP_CAN_ID, MotorType.kBrushless);
    motorWheelUp.setInverted(false);
    motorWheelUp.enableVoltageCompensation(12.0);
    pidWheelUp = new CANPIDController(motorWheelUp);
    pidWheelUp.setP(5e-4);
    pidWheelUp.setI(0.00e-8);
    pidWheelUp.setD(0);
    pidWheelUp.setIZone(0);
    pidWheelUp.setFF(0);
    pidWheelUp.setOutputRange(-5700, 5700);

    motorWheelDown = new CANSparkMax(RobotMap.LAUNCHER_MOTOR_DOWN_CAN_ID, MotorType.kBrushless);
    motorWheelDown.setInverted(true);
    motorWheelDown.enableVoltageCompensation(12.0);
    pidWheelDown = new CANPIDController(motorWheelDown);
    pidWheelDown.setP(1e-5);
    pidWheelDown.setI(0.0e-8);
    pidWheelDown.setD(0);
    pidWheelDown.setIZone(0);
    pidWheelDown.setFF(0);

    pidWheelDown.setOutputRange(-5700, 5700);

    ramp = new Solenoid(RobotMap.RAMP_SOLENOID_CHANNEL_ID);
  }

  public void shootPIDRPM() {
    pidWheelDown.setFF(TARGET_SPEED_DOWN * 12.0 / 5700.0 / 5700.0 / 10.0);
    pidWheelUp.setFF(TARGET_SPEED_UP * 12.0 / 5700.0 / 5700.0 / 2.0);
    pidWheelDown.setReference(TARGET_SPEED_DOWN, ControlType.kVelocity);
    pidWheelUp.setReference(TARGET_SPEED_UP, ControlType.kVelocity); // direction is always inverted in spark setup
                                                                     // (constructor)
  }

  public boolean isAtTargetSpeed() {
    double currentDownSpeed = motorWheelDown.getEncoder().getVelocity();
    double currentUpSpeed = motorWheelUp.getEncoder().getVelocity();
    double totalError = Math.abs(currentDownSpeed - TARGET_SPEED_DOWN) + Math.abs(currentUpSpeed - TARGET_SPEED_UP);
    return (totalError <= TOLERANCE_SPEED);
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
  }

  public void done() {

  }

  public void rampUp() {
    ramp.set(true);
  }

  public void rampDown() {
    ramp.set(false);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new KeepBallInRamp());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void log() {
    SmartDashboard.putNumber("Encoder Speed Up", motorWheelUp.getEncoder().getVelocity());
    SmartDashboard.putNumber("Encoder Speed Down", motorWheelDown.getEncoder().getVelocity());
  }

}
