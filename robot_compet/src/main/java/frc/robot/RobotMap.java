package frc.robot;

import javax.annotation.Nullable;

import com.google.auto.value.AutoValue;

@AutoValue
public abstract class RobotMap {

  // Joysticks
  public static final int JOYSTICK_DRIVER_PORT = 0;
  public static final int JOYSTICK_HELPER_PORT = 1;

  @SuppressWarnings("unused")
  private static final RobotMap CLONE_ROBOT_MAP = builder()
      .leftMotor1CanID(33)
      .leftMotor2CanID(34)
      .rightMotor1CanID(21)
      .rightMotor2CanID(22)
      .shifterSolenoidChannelID(null)
      .launcherMotorUpCanID(30)
      .launcherMotorDownCanID(31)
      .rampSolenoidChannelID(0)
      .intakeMotorCanID(null)
      .winchMotorCanID(null)
      .armExtenderMotorCanID(null)
      .pizzaTurnerCanID(null)
      .build();

  @SuppressWarnings("unused")
  private static final RobotMap COMPET_ROBOT_MAP = builder()
      .leftMotor1CanID(33)
      .leftMotor2CanID(34)
      .rightMotor1CanID(27)
      .rightMotor2CanID(28)
      .shifterSolenoidChannelID(7)
      .launcherMotorUpCanID(32)
      .launcherMotorDownCanID(29)
      .rampSolenoidChannelID(0)
      .intakeMotorCanID(23)
      .winchMotorCanID(25)
      .armExtenderMotorCanID(26)
      .pizzaTurnerCanID(null)
      .build();

  public static RobotMap instance() {
    // Changer selon le robot (clone ou competition):
    return COMPET_ROBOT_MAP;
  }

  // Drivetrain
  public abstract int leftMotor1CanID();

  public abstract int leftMotor2CanID();

  public abstract int rightMotor1CanID();

  public abstract int rightMotor2CanID();

  @Nullable
  public abstract Integer shifterSolenoidChannelID();

  // Launcher
  public abstract int launcherMotorUpCanID();

  public abstract int launcherMotorDownCanID();

  public abstract int rampSolenoidChannelID();

  // Intake
  public abstract Integer intakeMotorCanID();

  // Winch
  public abstract Integer winchMotorCanID();
  public abstract Integer armExtenderMotorCanID();
  
  // Pizza turner
  public abstract Integer pizzaTurnerCanID();

  private static Builder builder() {
    return new AutoValue_RobotMap.Builder();
  }

  @AutoValue.Builder
  public static abstract class Builder {
    public abstract Builder leftMotor1CanID(int leftMotor1CanID);

    public abstract Builder leftMotor2CanID(int leftMotor2CanID);

    public abstract Builder rightMotor1CanID(int rightMotor1CanID);

    public abstract Builder rightMotor2CanID(int rightMotor2CanID);

    public abstract Builder shifterSolenoidChannelID(@Nullable Integer shifterSolenoidChannelID);

    public abstract Builder launcherMotorUpCanID(int launcherMotorUpCanID);

    public abstract Builder launcherMotorDownCanID(int launcherMotorDownCanID);

    public abstract Builder rampSolenoidChannelID(int rampSolenoidChannelID);

    public abstract Builder intakeMotorCanID(@Nullable Integer intakeMotorCanID);

    public abstract Builder winchMotorCanID(@Nullable Integer winchMotorCanID);
    
    public abstract Builder armExtenderMotorCanID(@Nullable Integer armExtenderMotorCanID);

    public abstract Builder pizzaTurnerCanID(@Nullable Integer pizzaTurnerCanID);

    public abstract RobotMap build();
  }

  // Color Sensors
//  public static final int COLOR_SENSOR_MIDDLE_LEFT_ADDRESS = 0x10;
//  public static final int COLOR_SENSOR_REAR_LEFT_ADDRESS = 0x14;
//  public static final int COLOR_SENSOR_FRONT_LEFT_ADDRESS = 0x0;
//  public static final int COLOR_SENSOR_MIDDLE_RIGHT_ADDRESS = 0x0;
//  public static final int COLOR_SENSOR_REAR_RIGHT_ADDRESS = 0x0;
//  public static final int COLOR_SENSOR_FRONT_RIGHT_ADDRESS = 0x0;

}
