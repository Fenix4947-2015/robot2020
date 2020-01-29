/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.joysticks.XBoxButton;
import frc.robot.joysticks.XBoxJoystick;
import frc.robot.limelight.Limelight;

public class DriveArcade extends Command {

  private MoveMode moveMode = MoveMode.MANUAL;

  public Limelight m_limelight = new Limelight();

  public DriveArcade() {
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  private enum MoveMode {
    MANUAL, BACKWARD_TO_LINE, FORWARD_TO_LINE, OVERSHOOT_FORWARD, OVERSHOOT_BACKWARD, STAY_ON_LINE_FORWARD,
    STAY_ON_LINE_BACKWARD, STAY_ON_LINE_STOP
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    m_limelight.updateLimelightTracking();

    final boolean lockOnLineMode = XBoxJoystick.DRIVER.getButton(XBoxButton.B);

    double movePosValue = XBoxJoystick.DRIVER.getTriggerAxis(Hand.kRight, 0.05);
    double moveNegValue = XBoxJoystick.DRIVER.getTriggerAxis(Hand.kLeft, 0.05);
    double moveValue = movePosValue - moveNegValue;
    double rotateValue = XBoxJoystick.DRIVER.getX(Hand.kLeft, 0.05);

    // System.out.println("Move value: " + moveValue);
    // System.out.println("Rotate value: " + rotateValue);

    moveMode = MoveMode.MANUAL;
    
    boolean auto = XBoxJoystick.DRIVER.A.get();
    
    if (auto) {
      if (m_limelight.m_LimelightHasValidTarget) {
        Robot.driveTrain.driveArcadeMethod(-m_limelight.m_LimelightDriveCommand, m_limelight.m_LimelightSteerCommand);
            //arcadeDrive(0.0,-m_LimelightSteerCommand);
      }
      else {
        Robot.driveTrain.driveArcadeMethod(0.0, 0.0);
      }
    }
    else {
      Robot.driveTrain.driveArcadeMethod(-moveValue, rotateValue);
    }

    // Example of how to use the Pigeon IMU

    // 1. read the values from the sensor
    // Robot.driveTrain.pigeon.refresh();
    // 2. use the retrieved values
    // System.out.println("Yaw: " + Robot.driveTrain.pigeon.yaw);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
