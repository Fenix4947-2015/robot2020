/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.AutoAim;
import frc.robot.commands.drivetrain.DriveArcade;
import frc.robot.commands.launcher.RampMove;
import frc.robot.commands.launcher.RoutineShoot;
import frc.robot.joysticks.XBoxJoystick;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Launcher;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain _driveTrain = new DriveTrain();
  private final Launcher _launcher = new Launcher();

  private final AutoAim _autoAim = new AutoAim(_driveTrain);
  private final RoutineShoot _routineShoot = new RoutineShoot(_launcher);
  private final RampMove _rampMoveUp = new RampMove(_launcher, true);
  private final RampMove _rampMoveDown = new RampMove(_launcher, false);

  private final DriveArcade _driveArcade = new DriveArcade(_driveTrain);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    _driveTrain.setDefaultCommand(_driveArcade);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton autoAimButton = new JoystickButton(XBoxJoystick.DRIVER.getJoystick(),
        XboxController.Button.kA.value);
    JoystickButton shootButton = new JoystickButton(XBoxJoystick.DRIVER.getJoystick(), XboxController.Button.kB.value);
    JoystickButton rampButton = new JoystickButton(XBoxJoystick.DRIVER.getJoystick(), XboxController.Button.kY.value);

    autoAimButton.whenHeld(_autoAim);
    shootButton.whenPressed(_routineShoot);
    rampButton.whenPressed(_rampMoveUp);
    rampButton.whenReleased(_rampMoveDown);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
