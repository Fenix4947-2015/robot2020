package frc.robot.limelight;



import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;

public class Limelight {

    public boolean m_LimelightHasValidTarget = false;
    public double m_LimelightDriveCommand = 0.0;
    public double m_LimelightSteerCommand = 0.0;
    public PIDController pid_angle = new PIDController(0.024, 0.0052, 0.005);
  
    public void updateLimelightTracking() {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.03;                    // how hard to turn toward the target
        final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 0.025;        // Area of the target when the robot reaches the wall
        final double DESIRED_HEIGHT = 4.0;
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

        final double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    final double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    final double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    final double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    System.out.println(String.format("tv: %f, tx: %f, ty: %f, ta: %f", tv, tx, ty, ta));

    if (tv < 1.0) {
      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
      return;
    }

    m_LimelightHasValidTarget = true;
pid_angle.setSetpoint(0.0);
pid_angle.setTolerance(0.5);
double steer_cmd = pid_angle.calculate(-tx);
    // Start with proportional steering
   /* final double steerTarget = 0.0;
        double steer_cmd = 0.0;
        if(Math.abs(tx-steerTarget)<0.5)
        {
          steer_cmd =0.0;
        }
        else if(tx > steerTarget)
        {
          steer_cmd =0.50;
        }
        else if (tx < steerTarget)
        {
          steer_cmd =-0.50;
        }
        else{
          steer_cmd = tx * STEER_K;
          
        }
        */
        steer_cmd = steer_cmd  + Math.signum(steer_cmd)*0.175;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        //double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
        double drive_cmd = (DESIRED_HEIGHT - ty) * -DRIVE_K;

        steer_cmd = Math.max(steer_cmd, -0.5);
        drive_cmd = Math.max(drive_cmd, -0.5);

        steer_cmd = Math.min(steer_cmd, 0.5);
        drive_cmd = Math.min(drive_cmd, 0.5);

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE) {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
  }
}
