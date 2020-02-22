package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardSettings {

  private double _pidP = 0.024;
  private double _pidI = 0.0052;
  private double _pidD = 0.005;
  private double _pidF = 0.175;
  private String _pidType = "LLANGLE";

  public SmartDashboardSettings() {
    initSmartDashboard();
  }

  private void initSmartDashboard() {
    SmartDashboard.putNumber("pidP", _pidP);
    SmartDashboard.putNumber("pidI", _pidI);
    SmartDashboard.putNumber("pidD", _pidD);
    SmartDashboard.putNumber("pidF", _pidF);
    SmartDashboard.putString("pidType", _pidType);
  }

  public void refreshPidValues() {
    _pidP = SmartDashboard.getNumber("pidP", 0.024);
    _pidI = SmartDashboard.getNumber("pidI", 0.0052);
    _pidD = SmartDashboard.getNumber("pidD", 0.005);
    _pidF = SmartDashboard.getNumber("pidF", 0.175);
    _pidType = SmartDashboard.getString("pidType", "LLANGLE");
  }

  public String getPidType() {
    return _pidType;
  }

  public double getPidP() {
    return _pidP;
  }

  public double getPidI() {
    return _pidI;
  }

  public double getPidD() {
    return _pidD;
  }

  public double getPidF() {
    return _pidF;
  }

}
