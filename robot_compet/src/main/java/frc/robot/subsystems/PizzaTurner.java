/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class PizzaTurner extends SubsystemBase {
  /**
   * Creates a new PizzaTurner.
   */

  private final Solenoid _extender;
  private final CANSparkMax _spinner;

  public PizzaTurner() {
    if (RobotMap.PIZZA_TURNER_EXTENDER_CHANNEL_ID != null) {
      _extender = new Solenoid(RobotMap.PIZZA_TURNER_EXTENDER_CHANNEL_ID);
    } else {
      _extender = null;
    }

    if (RobotMap.PIZZA_TURNER_CAN_ID != null) {
      _spinner = new CANSparkMax(RobotMap.PIZZA_TURNER_CAN_ID, MotorType.kBrushless);
    } else {
      _spinner = null;
    }    
  }

  public void ExtendPizzaTurner()
  {
    _extender.set(true);
  }

  public void RetractPizzaTurner()
  {
    _extender.set(false);
  }

  public void StopPizzaTurner()
  {
    _spinner.stopMotor();
  }

  public void SpinPizza(double speed)
  {
    _spinner.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
