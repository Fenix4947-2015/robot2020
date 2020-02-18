/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class PreSpin extends Command {
  public PreSpin() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_Launcher);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(1.5);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Robot.m_Launcher.goToRPM(5100, 900);
    Robot.m_Launcher.openLoopShoot(true);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (isTimedOut());// || Robot.m_Launcher.GetIsAtTargetSpeed());
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
