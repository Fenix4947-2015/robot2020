package frc.robot.joysticks;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

public class DPadButton extends Button {

  private final XboxController joystick;
  private final Direction direction;

  public DPadButton(XboxController joystick, Direction direction) {
      this.joystick = joystick;
      this.direction = direction;
  }

  public static enum Direction {
      UP(0), RIGHT(90), DOWN(180), LEFT(270);

      int direction;

      private Direction(int direction) {
          this.direction = direction;
      }
  }

  @Override
  public boolean get() {
      int dPadValue = joystick.getPOV();
      return (dPadValue == direction.direction) || (dPadValue == (direction.direction + 45) % 360)
              || (dPadValue == (direction.direction + 315) % 360);
  }

}
