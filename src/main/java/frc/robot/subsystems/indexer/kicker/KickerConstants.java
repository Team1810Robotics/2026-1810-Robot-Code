package frc.robot.subsystems.indexer.kicker;

import static edu.wpi.first.units.Units.Amp;

import edu.wpi.first.units.measure.Current;

public class KickerConstants {
  public static final int KICKER_MOTOR = 12;

  public static final double SPINUP_TIME = .5;

  public static final Current JAM_CURRENT = Amp.of(38.5);

  public enum KickerState {
    IN(1),
    OUT(-1),
    STOP(0),
    SHOOTING(1);

    private final double power;

    private KickerState(double power) {
      this.power = power;
    }

    public double getPower() {
      return power;
    }
  }
}
