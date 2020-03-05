/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.SubCompressor;
import frc.robot.subsystems.Winch;

public class StopAllCommands extends CommandBase {
  private final DriveTrain _driveTrain;
  private final Intake _intake;
  private final SubCompressor _subCompressor;
  private final Launcher _launcher;
  private final Winch _winch;
  
  /**
   * Creates a new StopAllCommands.
   */
  public StopAllCommands(DriveTrain driveTrain, Intake intake, SubCompressor compressor, Launcher launcher, Winch winch) {
    _driveTrain = driveTrain;
    _intake = intake;
    _subCompressor = compressor;
    _launcher = launcher;
    _winch = winch;

    addRequirements(driveTrain);
    addRequirements(intake);
    addRequirements(compressor);
    addRequirements(launcher);
    addRequirements(winch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _driveTrain.driveArcadeMethod(0.0, 0.0);
    _intake.intakeStop();
    _launcher.stop();
    _winch.armExtendStop();
    _winch.winchStop();
    _subCompressor.disableCompressor();
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
