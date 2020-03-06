/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Winch extends SubsystemBase {
  /**
   * Creates a new Winch.
   */

  private final TalonSRX winch;
  private final TalonSRX armExtender;

  public Winch() {
    // TODO update with Robotmap
    if (RobotMap.WINCH_MOTOR_CAN_ID != null) {
      winch = new TalonSRX(RobotMap.WINCH_MOTOR_CAN_ID);
      winch.setInverted(false);
    } else {
      winch = null;
    }
    if (RobotMap.ARM_EXTENDER_MOTOR_CAN_ID != null) {
      armExtender = new TalonSRX(RobotMap.ARM_EXTENDER_MOTOR_CAN_ID);
      armExtender.setNeutralMode(NeutralMode.Coast);
          //new CANSparkMax(RobotMap.instance().armExtenderMotorCanID(), MotorType.kBrushless);
    } else {
      armExtender = null;
    }
  }

  public void winchStop() {
    if (winch != null) {
      winch.set(ControlMode.PercentOutput, 0.0);
    }
  }

  public void armExtendStop() {
    if (armExtender != null) {
      //armExtender.stopMotor();
      armExtender.set(ControlMode.PercentOutput, 0.0);
    }
  }

  public void winchLiftRobot(double Speed) {
    if (winch != null) {
      winch.set(ControlMode.PercentOutput, Speed);
    }
  }

  public void armExtend(double Speed) {
    if (armExtender != null) {
      armExtender.set(ControlMode.PercentOutput, Speed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public TalonSRX getArmExtenderTalonSRX() {
    return armExtender;
  }
}
