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
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class AutoNavigate extends CommandBase {
  public static final double K_FEED_FORWARD_ANGLE = 0.240;
  public static final double K_PID_P_ANGLE = 0.020;
  public static final double K_PID_I_ANGLE = 0.000;
  public static final double K_PID_D_ANGLE = 0.004;

  public static final double K_FEED_FORWARD_DISTANCE = 0.0;
  public static final double K_PID_P_DISTANCE = 0.5;
  public static final double K_PID_I_DISTANCE = 0.0;
  public static final double K_PID_D_DISTANCE = 0.07;
  public static final String PIDTYPE_AUTOAIM = "AUTOAIM";


  private final DriveTrain _driveTrain;

  private final SmartDashboardSettings _smartDashboardSettings;
  private final Intake _intake;
  private final boolean _withIntake;

  public double _driveCommand = 0.0;
  public double _steerCommand = 0.0;
  private double _distance = 0;
  private double _angle = 0;
  private PIDController _pidAngle = new PIDController(K_PID_P_ANGLE, K_PID_I_ANGLE, K_PID_D_ANGLE);
  private PIDController _pidDistance = new PIDController(K_PID_P_DISTANCE, K_PID_I_DISTANCE, K_PID_D_DISTANCE);
  private boolean _isAtSetPoint = false;
  private double _feedForward_angle = K_FEED_FORWARD_ANGLE;
  private double _feedForward_distance = K_FEED_FORWARD_DISTANCE;

  public AutoNavigate(DriveTrain driveTrain, Intake intake, SmartDashboardSettings smartDashboardSettings, double distance, double angle, boolean withIntake) {
    _driveTrain = driveTrain;
    _smartDashboardSettings = smartDashboardSettings;
    _distance = distance;
    _angle = angle;
    _isAtSetPoint = false;
    _intake = intake;
    _withIntake = withIntake;
    addRequirements(_driveTrain, _intake);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    //System.out.println("AutoNavigate " + _distance + " " + _angle);
    _driveTrain.shiftLow();
    _driveTrain.resetEncoders();
    if (_withIntake) {
      _intake.intakeStart(0.7);
    }
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
    if (Objects.equals(_smartDashboardSettings.getPidType(), "AUTONAV_ANGLE")) {
      setAnglePID(_smartDashboardSettings.getPidP(), _smartDashboardSettings.getPidI(),
          _smartDashboardSettings.getPidD(), _smartDashboardSettings.getPidF());
    }
    _smartDashboardSettings.refreshPidValues();
    if (Objects.equals(_smartDashboardSettings.getPidType(), "AUTONAV_DISTANCE")) {
      setDistancePID(_smartDashboardSettings.getPidP(), _smartDashboardSettings.getPidI(),
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
    _feedForward_angle = f;
  }

  public void setDistancePID(double p, double i, double d, double f) {
    // System.out.println(String.format("pid: %f %f %f %f", p, i, d, f));
    _pidDistance.setPID(p, i, d);
    _feedForward_distance = f;
  }

  public void updateTracking( double desired_distance,
   double desired_angle) {
    _driveCommand = 0.0;
    _steerCommand = 0.0;

    // These numbers must be tuned for your Robot! Be careful!
    final double MAX_DRIVE = 0.5; // Simple speed limit so we don't drive too fast

    final boolean tv =  true;
    final double tx = _driveTrain.getHeading();
    final double ty = _driveTrain.getAverageEncoderDistance();

    //System.out.println(String.format("tv: %s, tx: %f, ty: %f", tv, tx, ty));

    if (!tv) {
      return;
    }

    _pidAngle.setSetpoint(desired_angle);
    _pidAngle.setTolerance(2.5);
    double steer_cmd = _pidAngle.calculate(-tx);

    double feedFwd = Math.signum(steer_cmd) * _feedForward_angle;
    _steerCommand = steer_cmd + feedFwd;

    _pidDistance.setSetpoint(desired_distance);
    _pidDistance.setTolerance(0.5);
    double drive_cmd = _pidDistance.calculate(ty);

    feedFwd = Math.signum(drive_cmd) * _feedForward_distance;
    _driveCommand = drive_cmd + feedFwd;

    // try to drive forward until the target area reaches our desired area
    // double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
    //double drive_cmd = (desired_distance - ty) * -DRIVE_K;

    _steerCommand = Math.max(_steerCommand, -MAX_DRIVE);
    _driveCommand = Math.max(_driveCommand, -MAX_DRIVE);

    _steerCommand = Math.min(_steerCommand, MAX_DRIVE);
    _driveCommand = Math.min(_driveCommand, MAX_DRIVE);

    _isAtSetPoint = _pidAngle.atSetpoint() && _pidDistance.atSetpoint();
    //System.out.println(""+_pidAngle.atSetpoint() + "  " + _pidDistance.atSetpoint());
    
  }

}
