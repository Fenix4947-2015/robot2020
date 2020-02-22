/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeRollLowLevel extends CommandBase {
  /**
   * Creates a new IntakeRollLowLevel.
   */
  private Intake _intake;
  private boolean _spinInside;
  public IntakeRollLowLevel(Intake intake, boolean spinInside) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    _intake = intake;
    _spinInside = spinInside;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(_spinInside)
    {
      _intake.intakeStart(0.7);
    }
    else
    {
      _intake.intakeStop();
    }
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
