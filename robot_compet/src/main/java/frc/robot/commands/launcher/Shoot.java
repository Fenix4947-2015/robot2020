/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Launcher;

public class Shoot extends Command {
  private final Launcher launcher = Robot.m_Launcher;

  public Shoot() {
    System.out.println("RevPIDCommand() constructor");
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(launcher);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    launcher.setCurrentPhase("shoot");
    launcher.initLogging();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Robot.m_RevPIDSubsystem.goToPosition(angle);
    // System.out.println("RevPIDCommand.execute()");

    // Robot.m_Launcher.goToRPM(5100, 900);

    launcher.openLoopShoot(false);// .goToRPM(5100,900);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    launcher.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
