/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.*;

import frc.robot.joysticks.*;


/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class RobotLimelightSwerve extends TimedRobot {

  WPI_TalonSRX leftMaster = new WPI_TalonSRX(1);
  WPI_TalonSRX rightMaster = new WPI_TalonSRX(8);
    
  WPI_TalonSRX leftSlave1 = new WPI_TalonSRX(2);
  WPI_TalonSRX rightSlave1 = new WPI_TalonSRX(6);

  DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;

// This class is constructed in the robotInit method in Robot.java
public void Drivetrain() {
    leftSlave1.follow(leftMaster);
    rightSlave1.follow(rightMaster);
}

public void arcadeDrive(double move, double rotate) {
    drive.arcadeDrive(move, rotate);
}

  @Override
  public void robotInit() {
    Drivetrain();
  }

  @Override
  public void teleopPeriodic() {

    Update_Limelight_Tracking();


    double movePosValue = XBoxJoystick.DRIVER.getTriggerAxis(Hand.kRight, 0.05);
    double moveNegValue = XBoxJoystick.DRIVER.getTriggerAxis(Hand.kLeft, 0.05);
    double moveValue = movePosValue - moveNegValue;
    double rotateValue = XBoxJoystick.DRIVER.getX(Hand.kLeft, 0.05);
    boolean auto = XBoxJoystick.DRIVER.A.get();
    
    if (auto)
    {
      if (m_LimelightHasValidTarget)
      {
            arcadeDrive(m_LimelightDriveCommand,-m_LimelightSteerCommand);
            //arcadeDrive(0.0,-m_LimelightSteerCommand);
      }
      else
      {
            arcadeDrive(0.0,0.0);
      }
    }
    else
    {
      arcadeDrive(moveValue,rotateValue);
    }

    
  }

  public void Update_Limelight_Tracking()
  {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.03*8;                    // how hard to turn toward the target
        final double DRIVE_K = 0.26*5;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        //double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
        double drive_cmd = ty * DRIVE_K;

        steer_cmd = Math.max(steer_cmd, -0.5);
        drive_cmd = Math.max(drive_cmd, -0.5);

        steer_cmd = Math.min(steer_cmd, 0.5);
        drive_cmd = Math.min(drive_cmd, 0.5);

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
  }
}
