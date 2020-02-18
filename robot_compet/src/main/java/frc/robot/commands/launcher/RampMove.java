/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class RampMove extends Command {
  private final boolean _isUp;
  /// Up is true, down is false

  public RampMove(boolean isUp) {
    _isUp = isUp;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_Launcher);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(0.1);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (_isUp) {
      Robot.m_Launcher.rampUp();
    } else {
      Robot.m_Launcher.rampDown();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
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
