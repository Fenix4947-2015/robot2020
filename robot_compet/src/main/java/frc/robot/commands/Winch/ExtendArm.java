/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Winch;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.joysticks.XBoxJoystick;
import frc.robot.subsystems.Winch;

public class ExtendArm extends CommandBase {
  /**
   * Creates a new ExtendArm.
   */
  Winch _winch;
  public ExtendArm(Winch winch) {
    // Use addRequirements() here to declare subsystem dependencies.
    _winch = winch;
    addRequirements(winch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(_winch.endGameModeEnabled())
    {
      double extensionSpeed = 0.5 * XBoxJoystick.HELPER.getTriggerAxis(Hand.kRight,0.1);
      _winch.armExtend(extensionSpeed);      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _winch.armExtendStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
