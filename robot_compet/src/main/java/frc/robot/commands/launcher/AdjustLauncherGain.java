/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;

public class AdjustLauncherGain extends CommandBase {
  /**
   * Creates a new SlightBackSpin.
   */
  Launcher _launcher;
  final boolean _increment;
  public AdjustLauncherGain(Launcher launcher, boolean increment) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcher);
    _launcher  = launcher;
    _increment = increment;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double inc = _increment ? 0.01 : -0.01;
    _launcher.setDownWheelSpeed(_launcher.getDownWheelSpeed() + inc);
    SmartDashboard.putNumber("Launcher gain", _launcher.getDownWheelSpeed());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
