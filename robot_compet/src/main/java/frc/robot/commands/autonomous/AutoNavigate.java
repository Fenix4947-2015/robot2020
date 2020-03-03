/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import java.util.Objects;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SmartDashboardSettings;
import frc.robot.limelight.Limelight;
import frc.robot.subsystems.DriveTrain;

public class AutoNavigate extends CommandBase {
  public static final double K_FEED_FORWARD_ANGLE = 0.175;
  public static final double K_PID_P_ANGLE = 0.024;
  public static final double K_PID_I_ANGLE = 0.0052;
  public static final double K_PID_D_ANGLE = 0.005;

  public static final double K_FEED_FORWARD_DISTANCE = 0.0;
  public static final double K_PID_P_DISTANCE = 0.35;
  public static final double K_PID_I_DISTANCE = 0.0;
  public static final double K_PID_D_DISTANCE = 0.0;
  public static final String PIDTYPE_AUTOAIM = "AUTOAIM";

  private static final int AUTOAIM_PIPELINE = 0;

  private final DriveTrain _driveTrain;

  private final SmartDashboardSettings _smartDashboardSettings;

  public double _driveCommand = 0.0;
  public double _steerCommand = 0.0;
  private double _distance = 0;
  private double _angle = 0;
  private PIDController _pidAngle = new PIDController(K_PID_P_ANGLE, K_PID_I_ANGLE, K_PID_D_ANGLE);
  private PIDController _pidDistance = new PIDController(K_PID_P_DISTANCE, K_PID_I_DISTANCE, K_PID_D_DISTANCE);
  private boolean _isAtSetPoint = false;
  private double _feedForward = K_FEED_FORWARD_ANGLE;

  public AutoNavigate(DriveTrain driveTrain, SmartDashboardSettings smartDashboardSettings, double distance, double angle) {
    _driveTrain = driveTrain;
    _smartDashboardSettings = smartDashboardSettings;
    _distance = distance;
    _angle = angle;
    _isAtSetPoint = false;
    addRequirements(_driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    _driveTrain.shiftLow();
    _driveTrain.resetEncoders();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    refreshPidValues();
    updateTracking(_distance, _angle);
    _driveTrain.driveArcadeMethod(_driveCommand, _steerCommand);

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
    return _isAtSetPoint;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    _driveTrain.stop();
  }

  public void setAnglePID(double p, double i, double d, double f) {
    // System.out.println(String.format("pid: %f %f %f %f", p, i, d, f));
    _pidAngle.setPID(p, i, d);
    _feedForward = f;
  }

  public void updateTracking( double desired_distance,
   double desired_angle) {
    _driveCommand = 0.0;
    _steerCommand = 0.0;

    // These numbers must be tuned for your Robot! Be careful!
    final double STEER_K = 0.03; // how hard to turn toward the target
    final double DRIVE_K = 0.35; // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 0.025; // Area of the target when the robot reaches the wall
    final double MAX_DRIVE = 0.7; // Simple speed limit so we don't drive too fast

    final boolean tv =  true;
    final double tx = _driveTrain.getHeading();
    final double ty = _driveTrain.getAverageEncoderDistance();

    //System.out.println(String.format("tv: %s, tx: %f, ty: %f", tv, tx, ty));

    if (!tv) {
      return;
    }

    _pidAngle.setSetpoint(desired_angle);
    _pidAngle.setTolerance(0.25);
    double steer_cmd = _pidAngle.calculate(-tx);

    double feedFwd = Math.signum(steer_cmd) * _feedForward;
    _steerCommand = steer_cmd + feedFwd;

    _pidDistance.setSetpoint(desired_distance);
    _pidDistance.setTolerance(0.1);
    double drive_cmd = _pidDistance.calculate(ty);

    // try to drive forward until the target area reaches our desired area
    // double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
    //double drive_cmd = (desired_distance - ty) * -DRIVE_K;

    steer_cmd = Math.max(steer_cmd, -0.5);
    drive_cmd = Math.max(drive_cmd, -0.5);

    steer_cmd = Math.min(steer_cmd, 0.5);
    drive_cmd = Math.min(drive_cmd, 0.5);

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE) {
      drive_cmd = MAX_DRIVE;
    }
    _driveCommand = drive_cmd;

    _isAtSetPoint = _pidAngle.atSetpoint() && _pidDistance.atSetpoint();
  }

}
