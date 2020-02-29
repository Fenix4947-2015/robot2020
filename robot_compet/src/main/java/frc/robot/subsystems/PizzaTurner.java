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

  private final Solenoid extender;
  private final CANSparkMax spinner;

  public PizzaTurner() {
    if (RobotMap.instance().pizzaTurnerExtenderChannelID() != null) {
      extender = new Solenoid(RobotMap.instance().pizzaTurnerExtenderChannelID());
    } else {
      extender = null;
    }

    if (RobotMap.instance().pizzaTurnerCanID() != null) {
      spinner = new CANSparkMax(RobotMap.instance().pizzaTurnerCanID(), MotorType.kBrushless);
    } else {
      spinner = null;
    }    
  }

  public void ExtendPizzaTurner()
  {
    extender.set(true);
  }

  public void RetractPizzaTurner()
  {
    extender.set(false);
  }

  public void StopPizzaTurner()
  {
    spinner.stopMotor();
  }

  public void SpinPizza(double speed)
  {
    spinner.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
