/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevPIDSubsystem;

public class RevPIDCommand extends CommandBase {
  
  private final RevPIDSubsystem m_RevPIDSubsystem = new RevPIDSubsystem();

  private final double angle;
  private final double velocity;
  
  public RevPIDCommand(double position, double speed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    addRequirements(m_RevPIDSubsystem);
    angle = position;
    velocity = speed;
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      //setTimeout(1);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      // Robot.m_RevPIDSubsystem.goToPosition(angle);
      System.out.println("RevPIDCommand execute()");
      m_RevPIDSubsystem.goToRPM(velocity);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      //m_RevPIDSubsystem.done();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false; //isTimedOut();
    }

}
