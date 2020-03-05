package frc.robot;

public class RobotMap {

  public static final boolean COMPET = true;

  public static final boolean ALWAYS_WINCH = true;

  // Joysticks
  public static final int JOYSTICK_DRIVER_PORT = 0;
  public static final int JOYSTICK_HELPER_PORT = 1;

  public static final Integer LEFT_MOTOR1_CAN_ID;
  public static final Integer LEFT_MOTOR2_CAN_ID;
  public static final Integer RIGHT_MOTOR1_CAN_ID;
  public static final Integer RIGHT_MOTOR2_CAN_ID;
  public static final Integer SHIFTER_SOLENOID_CHANNEL_ID;
  public static final Integer LAUNCHER_MOTOR_UP_CAN_ID;
  public static final Integer LAUNCHER_MOTOR_DOWN_CAN_ID;
  public static final Integer RAMP_SOLENOID_CHANNEL_ID;
  public static final Integer INTAKE_MOTOR_CAN_ID;
  public static final Integer WINCH_MOTOR_CAN_ID;
  public static final Integer ARM_EXTENDER_MOTOR_CAN_ID;
  public static final Integer PIZZA_TURNER_CAN_ID;
  public static final Integer PIZZA_TURNER_EXTENDER_CHANNEL_ID;

  static {
    if (COMPET) {
      LEFT_MOTOR1_CAN_ID = 35;
      LEFT_MOTOR2_CAN_ID = 36;
      RIGHT_MOTOR1_CAN_ID = 27;
      RIGHT_MOTOR2_CAN_ID = 28;
      SHIFTER_SOLENOID_CHANNEL_ID = 1;
      LAUNCHER_MOTOR_UP_CAN_ID = 32;
      LAUNCHER_MOTOR_DOWN_CAN_ID = 29;
      RAMP_SOLENOID_CHANNEL_ID = 0;
      INTAKE_MOTOR_CAN_ID = 23;
      WINCH_MOTOR_CAN_ID = 25;
      ARM_EXTENDER_MOTOR_CAN_ID = 8; // talon
      PIZZA_TURNER_CAN_ID = 24;
      PIZZA_TURNER_EXTENDER_CHANNEL_ID = 2;
    } else {
      LEFT_MOTOR1_CAN_ID = 33;
      LEFT_MOTOR2_CAN_ID = 34;
      RIGHT_MOTOR1_CAN_ID = 21;
      RIGHT_MOTOR2_CAN_ID = 22;
      SHIFTER_SOLENOID_CHANNEL_ID = null;
      LAUNCHER_MOTOR_UP_CAN_ID = 30;
      LAUNCHER_MOTOR_DOWN_CAN_ID = 31;
      RAMP_SOLENOID_CHANNEL_ID = 0;
      INTAKE_MOTOR_CAN_ID = null;
      WINCH_MOTOR_CAN_ID = null;
      ARM_EXTENDER_MOTOR_CAN_ID = null;
      PIZZA_TURNER_CAN_ID = null;
      PIZZA_TURNER_EXTENDER_CHANNEL_ID = null;
    }
  }

}
