package frc.robot.subsystems.indexer;

public class IndexerConstants {
  public static byte SPIN_MOTOR = 13;
  public static byte KICKER_MOTOR = 12;

  public enum indexerState {
    IN(.4167, 1),
    OUT(-.3, -.1),
    STOP(0, 0);

    private double spinPower;
    private double kickPower;

    private indexerState(double spinPower, double kickPower) {
      this.spinPower = spinPower;
      this.kickPower = kickPower;
    }

    public double getSpinPower() {
      return spinPower;
    }

    public double getKickPower() {
      return kickPower;
    }
  }
}
