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

public class PreSpin extends CommandBase {
  private final Launcher _launcher;
  private final SubCompressor _compressor;
  private final boolean _far;

  public PreSpin(boolean far, Launcher launcher, SubCompressor compressor) {
    _launcher = launcher;
    _compressor = compressor;
    _far = far;
    addRequirements(launcher);
    addRequirements(compressor);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    System.out.println("PreSpin");
    _launcher.setCurrentPhase("prespin");
    _compressor.disableCompressor();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // Robot.m_Launcher.goToRPM(5100, 900);
    _launcher.openLoopShoot(true, _far);
    //_launcher.shootPIDRPM();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return _launcher.isAtTargetSpeed(_far);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

}
