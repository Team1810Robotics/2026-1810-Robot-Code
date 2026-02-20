package frc.robot.subsystems.indexer;

public class IndexerConstants {
  public static byte SPIN_MOTOR = 17;
  public static byte KICKER_MOTOR = 16;

  public enum indexerState {
    INT(1),
    OUT(-1),
    STOP(0);

    private double power;

    private indexerState(double power) {
      this.power = power;
    }

    public double getPower() {
      return power;
    }
  }
}
