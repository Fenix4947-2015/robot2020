/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.SubCompressor;

public class Shoot extends CommandBase {
  private final Launcher _launcher;
  private final SubCompressor _compressor;
  private final boolean _far;

  public Shoot(boolean far, Launcher launcher, SubCompressor compressor) {
    _launcher = launcher;
    _compressor = compressor;
    _far = far;
    addRequirements(launcher);
    addRequirements(compressor);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    _launcher.setCurrentPhase("shoot");
    _compressor.disableCompressor();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // Robot.m_RevPIDSubsystem.goToPosition(angle);
    // System.out.println("RevPIDCommand.execute()");

    // Robot.m_Launcher.goToRPM(5100, 900);

    //_launcher.openLoopShoot(false);// .goToRPM(5100,900);
    _launcher.openLoopShoot(false, _far);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    _launcher.stop();
    _compressor.enableCompressor();
  }
}
