/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.pizzaTurner;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.joysticks.XBoxJoystick;
import frc.robot.subsystems.PizzaTurner;

public class SpinPizza extends CommandBase {
  /**
   * Creates a new SpinPizza.
   */
  PizzaTurner _pizza;
  public SpinPizza(PizzaTurner pizza) {
    // Use addRequirements() here to declare subsystem dependencies.
    _pizza = pizza;
    addRequirements(pizza);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _pizza.ExtendPizzaTurner();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {    
    double speed = XBoxJoystick.HELPER.getX(Hand.kLeft, 0.1);
    _pizza.SpinPizza(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _pizza.StopPizzaTurner();
    _pizza.RetractPizzaTurner();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
