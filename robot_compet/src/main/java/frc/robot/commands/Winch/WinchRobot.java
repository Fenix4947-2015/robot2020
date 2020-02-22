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
import frc.robot.subsystems.SubCompressor;
import frc.robot.subsystems.Winch;

public class WinchRobot extends CommandBase {
  /**
   * Creates a new WinchRobot.
   */
  Winch _winch;
  SubCompressor _compressor;
  
  public WinchRobot(Winch winch, SubCompressor compressor) {
    _winch = winch;
    _compressor = compressor;
    addRequirements(winch);
    addRequirements(compressor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _compressor.disableCompressor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(_winch.endGameModeEnabled())
    {
      double liftingSpeed = 0.5 * XBoxJoystick.HELPER.getTriggerAxis(Hand.kLeft,0.1);
      _winch.armExtend(1.0);      // Todo : validate if we need to use copilot
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _winch.winchStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
