package frc.robot.subsystems.indexer.spindexer;

public class SpindexerConstants {
  public static final int SPIN_MOTOR = 13;

  public enum SpindexerState {
    IN(.8),
    OUT(-.4),
    STOP(0);

    private final double power;

    private SpindexerState(double power) {
      this.power = power;
    }

    public double getPower() {
      return power;
    }
  }
}
