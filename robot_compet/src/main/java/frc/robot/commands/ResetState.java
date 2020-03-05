package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class ResetState extends CommandBase {

  private final DriveTrain _driveTrain;
  private final Intake _intake;

  public ResetState(final DriveTrain driveTrain, final Intake intake) {
    _driveTrain = driveTrain;
    _intake = intake;
    addRequirements(driveTrain, intake);
  }

  @Override
  public void initialize() {
    System.out.println("ResetState");
    _driveTrain.resetEncoderAndHeading();
    _intake.intakeStop();
  }

  @Override
  public void execute() {
    System.out.println("ResetState execute()");
  }
  
  @Override
  public boolean isFinished() {
    return true;
  }
}
