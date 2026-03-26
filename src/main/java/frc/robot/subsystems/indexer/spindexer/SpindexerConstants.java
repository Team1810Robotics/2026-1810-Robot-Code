package frc.robot.subsystems.indexer.spindexer;

public class SpindexerConstants {
  public static final int SPIN_MOTOR = 13;

  public static final double kP = .0001;
  public static final double kS = .125;
  public static final double kV = .0021;

  public static final double SPINUP_TIME = .5;

  public enum SpindexerState {
    IN(2000),
    OUT(-3000),
    STOP(0),
    SHOOTING(0);

    private final double velocity;

    private SpindexerState(double velocity) {
      this.velocity = velocity;
    }

    public double getVelocity() {
      return velocity;
    }
  }
}
