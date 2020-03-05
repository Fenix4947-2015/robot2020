/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SmartDashboardSettings;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.PizzaTurner;
import frc.robot.subsystems.SubCompressor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class MoveFromLine extends SequentialCommandGroup {
  public MoveFromLine(DriveTrain driveTrain, Launcher launcher, PizzaTurner pizzaTurner,
      SmartDashboardSettings smartDashboardSettings, SubCompressor compressor, Intake intake) {
    super(new InitializeRobot(driveTrain, intake, launcher, pizzaTurner, compressor),
        new AutoNavigate(driveTrain, smartDashboardSettings, 2.0, 0.0).withTimeout(5.0));
  }
}
