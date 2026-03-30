package frc.robot.subsystems.shooter.flywheel;

public class FlywheelConstants {
  public static final int LEFT_FLYWHEEL_ID = 15;
  public static final int RIGHT_FLYWHEEL_ID = 14;

  public static final double BACKSPIN_RATIO = .95;

  public static final double frontkP = .7;
  public static final double frontkI = 0;
  public static final double frontkD = 0;
  public static final double frontkS = 0.7;
  public static final double frontkV = 0.115;

  public static final double backkP = .7;
  public static final double backkI = 0;
  public static final double backkD = 0;
  public static final double backkS = 0.7;
  public static final double backkV = 0.115;

  public enum FlywheelState {
    IDLE,
    SCORING,
    PASSING
  }
}
