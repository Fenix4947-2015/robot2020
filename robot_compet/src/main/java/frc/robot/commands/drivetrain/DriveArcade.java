/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.joysticks.XBoxJoystick;
import frc.robot.subsystems.DriveTrain;

public class DriveArcade extends CommandBase {

  private final DriveTrain _driveTrain;

  public DriveArcade(DriveTrain driveTrain) {
    _driveTrain = driveTrain;
    addRequirements(_driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    
    double movePosValue = XBoxJoystick.DRIVER.getTriggerAxis(Hand.kLeft, 0.075);
    double moveNegValue = XBoxJoystick.DRIVER.getTriggerAxis(Hand.kRight, 0.075);
    double moveValue = movePosValue - moveNegValue;
    double rotateValue = XBoxJoystick.DRIVER.getX(Hand.kLeft, 0.05);

    // System.out.println("Move value: " + moveValue);
    // System.out.println("Rotate value: " + rotateValue);

    _driveTrain.driveArcadeMethod(-moveValue * 0.8, 0.8*rotateValue);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
