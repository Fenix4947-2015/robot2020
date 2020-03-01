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
import frc.robot.commands.StopAllCommands;
import frc.robot.commands.Winch.ExtendArm;
import frc.robot.commands.Winch.WinchRobot;
import frc.robot.commands.autonomous.InitializeRobot;
import frc.robot.commands.autonomous.ShootLoaded;
import frc.robot.commands.compressor.CompressorDefault;
import frc.robot.commands.drivetrain.AutoAim;
import frc.robot.commands.drivetrain.DriveArcade;
import frc.robot.commands.drivetrain.Shift;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.intake.RollIntake;
import frc.robot.commands.launcher.RampMove;
import frc.robot.commands.launcher.RoutineShoot;
import frc.robot.commands.pizzaTurner.SpinPizza;
import frc.robot.joysticks.XBoxJoystick;
import frc.robot.limelight.Limelight;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.PizzaTurner;
import frc.robot.subsystems.SubCompressor;
import frc.robot.subsystems.Winch;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SmartDashboardSettings _smartDashboardSettings = new SmartDashboardSettings();

  // The robot's subsystems and commands are defined here...
  private final DriveTrain _driveTrain = new DriveTrain();
  private final Launcher _launcher = new Launcher(_smartDashboardSettings);
  private final Limelight _limelight = new Limelight();
  private final SubCompressor _compressor = new SubCompressor();
  private final Intake _intake = new Intake();
  private final Winch _winch = new Winch();
  private final PizzaTurner _pizzaTurner = new PizzaTurner();
  private final Pigeon _pigeon = new Pigeon(_winch.getArmExtenderTalonSRX());

  private final AutoAim _autoAim = new AutoAim(_driveTrain, _limelight, _smartDashboardSettings);
  private final RoutineShoot _routineShoot = new RoutineShoot(_launcher, _compressor, _intake);
  private final RampMove _rampMoveUp = new RampMove(_launcher, true);
  private final RampMove _rampMoveDown = new RampMove(_launcher, false);

  private final RollIntake _rollIntake = new RollIntake(_intake);
  private final ReverseIntake _reverseIntake = new ReverseIntake(_intake);
  private final ExtendArm _extendArm = new ExtendArm(_winch,_pizzaTurner);
  private final WinchRobot _winchRobot = new WinchRobot(_winch, _compressor);

  private final DriveArcade _driveArcade = new DriveArcade(_driveTrain);
  private final Shift _shiftHigh = new Shift(_driveTrain, true);
  private final Shift _shiftLow = new Shift(_driveTrain, false);

  private final SpinPizza _spinPizza = new SpinPizza(_pizzaTurner);

  private final InitializeRobot _initializeRobot = new InitializeRobot(_driveTrain, _launcher, _pizzaTurner);
  private final StopAllCommands _stopAllCommands = new StopAllCommands(_driveTrain,_intake,_compressor,_launcher,_winch);
  private final CompressorDefault _compressorDefault = new CompressorDefault(_compressor);

  private final ShootLoaded _shootLoaded = new ShootLoaded(_driveTrain, _launcher, _pizzaTurner, _limelight, _smartDashboardSettings, _compressor, _intake);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    _driveTrain.setDefaultCommand(_driveArcade);
    _compressor.setDefaultCommand(_compressorDefault);
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
    final XboxController driverController = XBoxJoystick.DRIVER.getJoystick();
    final XboxController helperController = XBoxJoystick.HELPER.getJoystick();
    
    // DRIVER COMMANDS
    JoystickButton shiftHighButton = new JoystickButton(driverController, XboxController.Button.kBumperLeft.value);
    JoystickButton shiftLowButton = new JoystickButton(driverController, XboxController.Button.kBumperRight.value);
    // joystick gauche driver pour l'orientation et les 2 gachettes pour la vitesse. 
    JoystickButton autoAimButton = new JoystickButton(driverController, XboxController.Button.kA.value);
    JoystickButton shootButton = new JoystickButton(driverController, XboxController.Button.kB.value);
    JoystickButton reverseIntakeButton = new JoystickButton(driverController, XboxController.Button.kY.value);
    JoystickButton intakeButton = new JoystickButton(driverController, XboxController.Button.kX.value);
    
    JoystickButton resetAllRobotButton = new JoystickButton(driverController, XboxController.Button.kStart.value);
    JoystickButton initializeRobotButton = new JoystickButton(driverController, XboxController.Button.kBack.value);
    

    // HELPER COMMANDS
    JoystickButton winchButton = new JoystickButton(helperController, XboxController.Button.kBumperLeft.value);
    JoystickButton extendArmButton = new JoystickButton(helperController, XboxController.Button.kBack.value);   
    JoystickButton spinPizzaButton = new JoystickButton(helperController, XboxController.Button.kA.value);

    // y driver reverse intake
    // back updown right joystick helper pour arm extend.

    autoAimButton.whenHeld(_autoAim);
    shootButton.whenPressed(_routineShoot);
    intakeButton.whenHeld(_rollIntake);
    reverseIntakeButton.whileHeld(_reverseIntake);

    winchButton.whileHeld(_winchRobot);
    extendArmButton.whileHeld(_extendArm); // todo change to good function

    shiftHighButton.whenPressed(_shiftHigh);
    shiftLowButton.whenPressed(_shiftLow);

    resetAllRobotButton.whenPressed(_stopAllCommands);
    initializeRobotButton.whenPressed(_initializeRobot);

    spinPizzaButton.whileHeld(_spinPizza);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    // An ExampleCommand will run in autonomous
    return _shootLoaded;
  }
}
