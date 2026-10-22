package frc.robot.subsystems.shooter.stationary;

public class StaticConstants {
  // HOOD//
  public static final int HOOD_MOTOR_ID = 16;
  public static final int HOOD_ENCODER_DIO = 35; // /CHECK THIS BS

  public static final int MINDEGREE_HOOD_ANGLE = 0;
  public static final int MAXDEGREE_HOOD_ANGLE = 30;

  public static final double POSITION1 = 0.195;
  public static final double POSITION2 = 1.200;
  public static final double POSITION3 = 2.244; // /CHECK THIS BS

  public static final double DEFAULT_POSITION = POSITION2;

  public static final double HOOD_SPEED = 0.25;

  public static final double kP = 0.8; // modify :(
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  // Max output used both for PID and manual mode clamping
  public static final double MAX_PID_OUTPUT = 0.35;

  // How close is "good enough"
  public static final double HOOD_TOLERANCE = 0.03;

  // Wrap detection threshold for a 0-1 absolute encoder
  public static final double ENCODER_WRAP_THRESHOLD = 0.5;
}
