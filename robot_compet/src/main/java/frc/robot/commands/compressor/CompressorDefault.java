/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.compressor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubCompressor;

public class CompressorDefault extends CommandBase {
  /**
   * Creates a new CompressorDefault.
   */
  private SubCompressor _compressor;

  public CompressorDefault(SubCompressor compressor) {
    // Use addRequirements() here to declare subsystem dependencies.
    _compressor = compressor;
    addRequirements(compressor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _compressor.enableCompressor();
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