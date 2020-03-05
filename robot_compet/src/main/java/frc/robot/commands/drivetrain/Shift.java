/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Shift extends CommandBase {
  /**
   * Creates a new ShiftHigh.
   */
  private final DriveTrain _driveTrain;
  private final boolean _high;

  public Shift(final DriveTrain driveTrain, boolean high) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveTrain = driveTrain;
    _high = high;
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("Shift");
    
    if (_high) {
      _driveTrain.shiftHigh();
    } else {
      _driveTrain.shiftLow();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
