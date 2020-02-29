package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {

  private final CANSparkMax intake;

  public Intake() {
    // TODO update with Robotmap
    if (RobotMap.instance().intakeMotorCanID() != null) {
      intake = new CANSparkMax(RobotMap.instance().intakeMotorCanID(), MotorType.kBrushless);
      intake.setIdleMode(IdleMode.kBrake);
    } else {
      intake = null;
    }
  }

  public void intakeStop() {
    if (intake != null) {
      intake.stopMotor();
    }
  }

  public void intakeStart(double Speed) {
    if (intake != null) {
      intake.set(Speed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
