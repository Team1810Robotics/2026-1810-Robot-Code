package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static byte LEFT_MOTOR = 13;
  public static byte RIGHT_MOTOR = 14; // intake motor is 15
  public static byte LEFT_ENCODER_ID = 14;
  public static byte RIGHT_ENCODER_ID = 14;
  public static byte ROLLER_MOTOR = 15;

  public static double kP = 0;
  public static double kI = 0;
  public static double kD = 0;

  public static double RIGHT_ENCODER_OFFSET = 0;
  public static double LEFT_ENCODER_OFFSET = 0;

  public enum intakeState {
    INTAKE(1),
    OUT(-1),
    STOP(0);

    private double power;

    private intakeState(double power) {
      this.power = power;
    }

    public double getPower() {
      return power;
    }
  }
}

// I'm special.
