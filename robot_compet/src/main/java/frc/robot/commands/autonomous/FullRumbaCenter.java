/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SmartDashboardSettings;
import frc.robot.commands.drivetrain.AutoAim;
import frc.robot.commands.launcher.RoutineShoot;
import frc.robot.limelight.Limelight;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.PizzaTurner;
import frc.robot.subsystems.SubCompressor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FullRumbaCenter extends SequentialCommandGroup {
  public FullRumbaCenter(DriveTrain driveTrain, Launcher launcher, PizzaTurner pizzaTurner, Limelight limelight,
      SmartDashboardSettings smartDashboardSettings, SubCompressor compressor, Intake intake) {
    super(new InitializeRobot(driveTrain, intake, launcher, pizzaTurner, compressor),
        new AutoNavigate(driveTrain, intake, smartDashboardSettings, 9.0, 0.0, false).withTimeout(3.0),
        new AutoNavigate(driveTrain, intake, smartDashboardSettings, 0.0, -80.0, false).withTimeout(2.0),
        new AutoNavigate(driveTrain, intake, smartDashboardSettings, 2.5, 0.0, true).withTimeout(5.0),
        new AutoNavigate(driveTrain, intake, smartDashboardSettings, -2.5, 0.0, true).withTimeout(5.0),
        new AutoNavigate(driveTrain, intake, smartDashboardSettings, 0.0, -150.0, false).withTimeout(5.0),
        new AutoAim(AutoAim.AUTOAIM_FAR_PIPELINE, driveTrain, limelight, smartDashboardSettings).withTimeout(3.0),
        new RoutineShoot(RoutineShoot.FAR, launcher, compressor, intake));
  }
}
