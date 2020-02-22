/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.limelight.Limelight;
import frc.robot.subsystems.DriveTrain;

public class AutoAim extends CommandBase {
  private final DriveTrain _driveTrain;
  private final Limelight m_limelight = new Limelight();

  public AutoAim(DriveTrain driveTrain) {
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
    m_limelight.updateLimelightTracking();
    if (m_limelight.m_LimelightHasValidTarget) {
      _driveTrain.driveArcadeMethod(-m_limelight.m_LimelightDriveCommand, m_limelight.m_LimelightSteerCommand);
    } else {
      _driveTrain.stop();
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    _driveTrain.stop();
  }
}
