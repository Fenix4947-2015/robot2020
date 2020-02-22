/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Launcher;

public class RoutineShoot extends SequentialCommandGroup {

  //private final Launcher _launcher;

  public RoutineShoot(Launcher launcher) {
    //_launcher = launcher;
    addRequirements(launcher);

    addCommands(new PreSpin(launcher).withTimeout(1.0), new RampMove(launcher, true).withTimeout(0.1),
        (new Shoot(launcher)).withTimeout(2.5), new RampMove(launcher, false).withTimeout(0.1));
  }
}
