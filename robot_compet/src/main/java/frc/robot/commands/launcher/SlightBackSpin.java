/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;

public class SlightBackSpin extends CommandBase {
  /**
   * Creates a new SlightBackSpin.
   */
  Launcher _launcher;
  public SlightBackSpin(Launcher launcher) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcher);
    _launcher  = launcher;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _launcher.LowerWheelSlightBackspin();
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
    return false;
  }
}
