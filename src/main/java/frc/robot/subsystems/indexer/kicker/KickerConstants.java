package frc.robot.subsystems.indexer.kicker;

public class KickerConstants {
  public static int KICKER_MOTOR = 12;

  public enum KickerState {
    IN(1),
    OUT(-1),
    STOP(0);

    private final double power;

    private KickerState(double power) {
      this.power = power;
    }

    public double getPower() {
      return power;
    }
  }
}
