/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  @Override
  public void robotInit() {
    WPI_TalonSRX left1 = new WPI_TalonSRX(1);
    WPI_TalonSRX left2 = new WPI_TalonSRX(2);
    WPI_TalonSRX right1 = new WPI_TalonSRX(6);
    WPI_TalonSRX right2 = new WPI_TalonSRX(8);
    right2.follow(right1);
    left2.follow(left1);
    m_myRobot = new DifferentialDrive(left1, right1);
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(4);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.arcadeDrive(-m_leftStick.getY(), m_leftStick.getX());
  }
}
