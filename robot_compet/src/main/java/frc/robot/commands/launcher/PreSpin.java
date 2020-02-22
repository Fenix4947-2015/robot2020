/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;

public class PreSpin extends CommandBase {
  private final Launcher launcher;

  public PreSpin(Launcher launcher) {
    this.launcher = launcher;
    addRequirements(launcher);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    launcher.setCurrentPhase("prespin");
    launcher.initLogging();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // Robot.m_Launcher.goToRPM(5100, 900);
    launcher.openLoopShoot(true);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;// || Robot.m_Launcher.GetIsAtTargetSpeed());
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

}
