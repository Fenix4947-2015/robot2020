/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import java.util.Objects;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SmartDashboardSettings;
import frc.robot.limelight.Limelight;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class BallPickup extends CommandBase {
  public static final double K_FEED_FORWARD = 0.41;
  public static final double K_PID_P = 0.018;
  public static final double K_PID_I = 0.000;
  public static final double K_PID_D = 0.002;
  public static final String PIDTYPE_AUTOAIM = "AUTOAIM";

  private static final int PICKUP_PIPELINE = 1;

  private final DriveTrain _driveTrain;
  private final Limelight _limelight;
  private final Intake _intake;

  private final SmartDashboardSettings _smartDashboardSettings;

  public double _driveCommand = 0.0;
  public double _steerCommand = 0.0;
  private PIDController _pidAngle = new PIDController(K_PID_P, K_PID_I, K_PID_D);
  private double _feedForward = K_FEED_FORWARD;

  private boolean _ballInRange = false;
  private boolean _ballInIntake = false;

  public BallPickup(DriveTrain driveTrain, Limelight limelight, Intake intake, SmartDashboardSettings smartDashboardSettings) {
    _driveTrain = driveTrain;
    _limelight = limelight;
    _smartDashboardSettings = smartDashboardSettings;
    _intake = intake;

    addRequirements(_driveTrain, _limelight, _intake);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    _ballInRange = false;
    _ballInIntake = false;

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    _limelight.changePipeline(PICKUP_PIPELINE);
    refreshPidValues();
    updateTracking();
    _intake.intakeStart(1.0);

    if (_ballInRange == true) {
      _driveTrain.driveArcadeMethod(0.3, 0);
      if(_driveTrain.getAverageEncoderDistance() > 1.5){
        _ballInIntake = true;
        _ballInRange = false;

      }
    } else if (_limelight.isTargetValid()){
      _driveTrain.driveArcadeMethod(-_driveCommand, _steerCommand);
    }   else {
      _driveTrain.stop();
    }


  }

  private void refreshPidValues() {
    _smartDashboardSettings.refreshPidValues();
    if (Objects.equals(_smartDashboardSettings.getPidType(), PIDTYPE_AUTOAIM)) {
      setAnglePID(_smartDashboardSettings.getPidP(), _smartDashboardSettings.getPidI(),
          _smartDashboardSettings.getPidD(), _smartDashboardSettings.getPidF());
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return _ballInIntake;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    _driveTrain.stop();
    _intake.intakeStop();

  }

  public void setAnglePID(double p, double i, double d, double f) {
    // System.out.println(String.format("pid: %f %f %f %f", p, i, d, f));
    _pidAngle.setPID(p, i, d);
    _feedForward = f;
  }

  public void updateTracking() {
    _driveCommand = 0.0;
    _steerCommand = 0.0;

    // These numbers must be tuned for your Robot! Be careful!
    final double STEER_K = 0.03; // how hard to turn toward the target
    final double DRIVE_K = 0.35; // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 0.025; // Area of the target when the robot reaches the wall
    final double DESIRED_HEIGHT = 0.0; //8.6;
    final double DESIRED_ANGLE = 2.5; // 0.0; // 2.5 is a patch for misaligned limelight!! TODO realign
    final double MAX_DRIVE = 0.3; // Simple speed limit so we don't drive too fast

    final boolean tv = _limelight.isTargetValid();
    final double tx = _limelight.getTx();
    final double ty = _limelight.getTy();

    //System.out.println(String.format("tv: %s, tx: %f, ty: %f", tv, tx, ty));

    if (!tv) {
      return;
    }

    _pidAngle.setSetpoint(DESIRED_ANGLE);
    _pidAngle.setTolerance(0.25);
    double steer_cmd = _pidAngle.calculate(-tx);

    // Start with proportional steering
//    final double steerTarget = 0.0;
//    double steer_cmd = 0.0;
//    if (Math.abs(tx - steerTarget) < 0.5) {
//      steer_cmd = 0.0;
//    } else if (tx > steerTarget) {
//      steer_cmd = 0.50;
//    } else if (tx < steerTarget) {
//      steer_cmd = -0.50;
//    } else {
//      steer_cmd = tx * STEER_K;
//    }

    double feedFwd = Math.signum(steer_cmd) * _feedForward;
    _steerCommand = steer_cmd + feedFwd;

    // try to drive forward until the target area reaches our desired area
    // double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
    double drive_cmd = (DESIRED_HEIGHT - ty) * DRIVE_K;

    steer_cmd = Math.max(steer_cmd, -0.5);
    drive_cmd = Math.max(drive_cmd, -0.5);

    steer_cmd = Math.min(steer_cmd, 0.5);
    drive_cmd = Math.min(drive_cmd, 0.5);

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE) {
      drive_cmd = MAX_DRIVE;
    }
    _driveCommand = drive_cmd;
  if (ty < 0.0) {

    _ballInRange = true;
    _driveTrain.resetEncoders();

  }
  
  }

}
