package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static byte LEFT_MOTOR = 13;
  public static byte RIGHT_MOTOR = 14;
  public static byte LEFT_ENCODER_ID = 14;
  public static byte RIGHT_ENCODER_ID = 14;
  public static byte ROLLER_MOTOR = 15;

  public static double kP = 0;
  public static double kI = 0;
  public static double kD = 0;

  public static double RIGHT_ENCODER_OFFSET = 0;
  public static double LEFT_ENCODER_OFFSET = 0;

  public enum rollerState {
    INTAKE(1),
    OUT(-1),
    STOP(0);

    private double power;

    private rollerState(double power) {
      this.power = power;
    }

    public double getPower() {
      return power;
    }
  }

  public enum deployState {
    DEPLOY(0),
    RETRACT(0);

    private double position;

    private deployState(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }

  public enum IntakeState {
    OUT_INTAKE(rollerState.INTAKE, deployState.DEPLOY),
    OUT_STOP(rollerState.STOP, deployState.DEPLOY),
    RETRACT(rollerState.STOP, deployState.RETRACT);

    private final rollerState roller;
    private final deployState deploy;

    private IntakeState(rollerState roller, deployState deploy) {
      this.roller = roller;
      this.deploy = deploy;
    }

    public rollerState getRollerState() {
      return this.roller;
    }

    public deployState getDeployState() {
      return this.deploy;
    }
  }
}

// I'm special.
