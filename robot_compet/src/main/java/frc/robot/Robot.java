/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.limelight.Limelight;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Launcher;

/**
 * 
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // private Command m_autonomousCommand;

  // private RobotContainer m_robotContainer;
  public static Launcher m_Launcher = new Launcher();

  // public static RevPIDCommand m_RevPIDCommand = new RevPIDCommand(0.0, 0.0);
  // public static RevSRX m_RevSRX = new RevSRX();
  public static DriveTrain driveTrain;
  public static Limelight limeLight;

  public static OI oi;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    // m_robotContainer = new RobotContainer();
    driveTrain = new DriveTrain();
    limeLight = new Limelight();
    oi = new OI();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    // CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    // m_autonomousCommand.schedule();
    // }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    // m_autonomousCommand.cancel();
    // }
    System.out.println("Robot.teleopInit()");
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    log();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();
    log();
    getPidValues();
    updatePidValues();
  }

  private void log() {
    driveTrain.log();
    //m_Launcher.log();
  }

  private Double _pidP;
  private Double _pidI;
  private Double _pidD;
  private Double _pidF;
  private String _pidType;
  
  public void initSmartDashboard(){
    SmartDashboard.putNumber("pidP", 1.0);
    SmartDashboard.putNumber("pidI", 1.0);
    SmartDashboard.putNumber("pidD", 1.0);
    SmartDashboard.putNumber("pidF", 1.0);
    SmartDashboard.putString("pidType", "LLANGLE");
  }

  public void getPidValues(){
    _pidP = SmartDashboard.getNumber("pidP", 1.0);
    _pidI = SmartDashboard.getNumber("pidI", 1.0);
    _pidD = SmartDashboard.getNumber("pidD", 1.0);
    _pidF = SmartDashboard.getNumber("pidF", 1.0);
    _pidType = SmartDashboard.getString("pidType", "LLANGLE");
  }

  public void updatePidValues(){
    if(_pidType == "LLANGLE"){
      limeLight.updateAnglePID(_pidP, _pidI, _pidD, _pidF);
    }
  }

}
