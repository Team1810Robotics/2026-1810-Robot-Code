package frc.robot.subsystems.indexer.spindexer;

public class SpindexerConstants {
  public static final int SPIN_MOTOR = 13;

  public static final double kP = .0001;
  public static final double kS = .125;
  public static final double kV = .0021;

  public static final double SPINUP_TIME = .5;

  public enum SpindexerState {
    IN(.3),
    OUT(-.6),
    STOP(0),
    SHOOTING(.3);

    private final double power;

    private SpindexerState(double power) {
      this.power = power;
    }

    public double getPower() {
      return power;
    }
  }
}
