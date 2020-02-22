/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;

public class RampMove extends CommandBase {
  private final Launcher _launcher;
  private final boolean _isUp;
  /// Up is true, down is false

  public RampMove(Launcher launcher, boolean isUp) {
    _launcher = launcher;
    _isUp = isUp;
    // Use requires() here to declare subsystem dependencies
    addRequirements(_launcher);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    // setTimeout(0.1);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (_isUp) {
      _launcher.rampUp();
    } else {
      _launcher.rampDown();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false; // isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
