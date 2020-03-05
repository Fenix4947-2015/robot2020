/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Winch;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.joysticks.XBoxJoystick;
import frc.robot.subsystems.PizzaTurner;
import frc.robot.subsystems.Winch;

public class ExtendArm extends CommandBase {
  /**
   * Creates a new ExtendArm.
   */
  private final Winch _winch;
  private final PizzaTurner _pizzaTurner;
  private final RobotContainer _robotContainer;

  public ExtendArm(Winch winch, PizzaTurner pizzaTurner, RobotContainer robotContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    _winch = winch;
    _pizzaTurner = pizzaTurner;
    _robotContainer = robotContainer;
    addRequirements(winch);
    addRequirements(pizzaTurner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotMap.ALWAYS_WINCH || _robotContainer.getGameState().winchAllowed) {
      _pizzaTurner.ExtendPizzaTurner();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotMap.ALWAYS_WINCH || _robotContainer.getGameState().winchAllowed) {
      double extensionSpeed = XBoxJoystick.HELPER.getY(Hand.kRight, 0.1);
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
