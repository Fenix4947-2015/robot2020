/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ResetState;
import frc.robot.commands.drivetrain.Shift;
import frc.robot.commands.pizzaTurner.RetractPizzaTurner;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.PizzaTurner;
import frc.robot.subsystems.SubCompressor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class InitializeRobot extends SequentialCommandGroup {
  /**
   * Creates a new InitializeRobot.
   */
  public InitializeRobot(final DriveTrain driveTrain, final Intake intake, final Launcher launcher,
      final PizzaTurner pizzaTurner, SubCompressor compressor) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    addCommands(new ResetState(driveTrain, intake),
        new RetractPizzaTurner(pizzaTurner), new Shift(driveTrain, false),
        new TimedMove(driveTrain, 1.0).withTimeout(0.5), new TimedMove(driveTrain, -1.0).withTimeout(0.5)
        );
    }
}
