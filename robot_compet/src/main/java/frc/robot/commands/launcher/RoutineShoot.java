/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeRollLowLevel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.SubCompressor;

public class RoutineShoot extends SequentialCommandGroup {

  public static final boolean FAR = true;
  public static final boolean NEAR = false;

  public RoutineShoot(boolean far, Launcher launcher, SubCompressor compressor, Intake intake) {
    // _launcher = launcher;

    addCommands(new PreSpin(far, launcher, compressor).withTimeout(1.0), new IntakeRollLowLevel(intake, true),
        new RampMove(launcher, true).withTimeout(0.1), new Shoot(far, launcher, compressor).withTimeout(3.25),
        new RampMove(launcher, false).withTimeout(0.1), new IntakeRollLowLevel(intake, false));
  }
}
