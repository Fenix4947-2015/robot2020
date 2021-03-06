package frc.robot;

public enum GameState {
  UNKNOWN(false),
  AUTONOMOUS(false),
  TELEOP(true),
  TELEOP_ENDGAME(true),
  TEST(true);
  
  public final boolean winchAllowed;
  
  private GameState(boolean winchAllowed) {
    this.winchAllowed = winchAllowed;
  }

}
