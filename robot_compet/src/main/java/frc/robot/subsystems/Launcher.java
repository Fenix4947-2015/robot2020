/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Launcher.KeepBallInRamp;
import frc.robot.commands.Launcher.Shoot;

/**
 * Add your docs here.
 */
public class Launcher extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private CANSparkMax MotorWheelUp;
  private CANPIDController PidWheelUp;
  private CANSparkMax MotorWheelDown;
  private CANPIDController PidWheelDown;
  private Solenoid ramp;

  private double _speedTargetDown = 0.0;
  private double _speedTargetUp = 0.0;
  private static final double _toleranceSpeed = 10;

  public Launcher() {
    System.out.println("Launcher constructor");

    MotorWheelUp = new CANSparkMax(30, MotorType.kBrushless);
    MotorWheelUp.setInverted(false);
    MotorWheelUp.enableVoltageCompensation(12.0);
    PidWheelUp = new CANPIDController(MotorWheelUp);
    PidWheelUp.setP(5e-4);
    PidWheelUp.setI(0.00e-8);
    PidWheelUp.setD(0);
    PidWheelUp.setIZone(0);
    PidWheelUp.setFF(0);    
    PidWheelUp.setOutputRange(-5700, 5700);

    MotorWheelDown = new CANSparkMax(31, MotorType.kBrushless);
    MotorWheelDown.setInverted(true);
    MotorWheelDown.enableVoltageCompensation(12.0);
    PidWheelDown = new CANPIDController(MotorWheelDown);
    PidWheelDown.setP(1e-5);
    PidWheelDown.setI(0.0e-8);
    PidWheelDown.setD(0);
    PidWheelDown.setIZone(0);
    PidWheelDown.setFF(0);
    
    PidWheelDown.setOutputRange(-5700, 5700);

    ramp = new Solenoid(0);
  }

    
  public void ShootPIDRPM()
  {
    _speedTargetDown = 900;
    _speedTargetUp = 5100;
    PidWheelDown.setFF(_speedTargetDown * 12.0 / 5700.0 /5700.0 / 10.0);
    PidWheelUp.setFF(_speedTargetUp * 12.0/ 5700.0 /5700.0 /2.0 );
    PidWheelDown.setReference(_speedTargetDown, ControlType.kVelocity);
    PidWheelUp.setReference(_speedTargetUp, ControlType.kVelocity); // direction is always inverted in spark setup (constructor)
  }

  public boolean GetIsAtTargetSpeed()
  {
    double currentDownSpeed = MotorWheelDown.getEncoder().getVelocity();
    double currentUpSpeed = MotorWheelUp.getEncoder().getVelocity();
    double totalError = Math.abs(currentDownSpeed - _speedTargetDown) + Math.abs(currentUpSpeed - _speedTargetUp);
    return (totalError <=_toleranceSpeed);
  }

  public void OpenLoopShoot()
  {
    MotorWheelUp.set(0.15);
    MotorWheelDown.set(0.80);
  }

  public void Stop()
  {
    MotorWheelUp.set(0.0);
    MotorWheelDown.set(0.0);
  }

  public void done() {

   
  }

  public void RampUp()
  {
    ramp.set(true);
  }

  public void RampDown()
  {
    ramp.set(false);
  }

  public void initDefaultCommand() {
    setDefaultCommand(new KeepBallInRamp());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void log()
  {
    SmartDashboard.putNumber("Encoder Speed Up", MotorWheelUp.getEncoder().getVelocity());
    SmartDashboard.putNumber("Encoder Speed Down", MotorWheelDown.getEncoder().getVelocity());
  }

}
