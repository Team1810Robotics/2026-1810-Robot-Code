package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public class TunablePIDF {
  private final DoubleSubscriber kPSubscriber;
  private final DoubleSubscriber kISubscriber;
  private final DoubleSubscriber kDSubscriber;
  private final DoubleSubscriber kSSubscriber;
  private final DoubleSubscriber kGSubscriber;
  private final DoubleSubscriber kVSubscriber;
  private final DoubleSubscriber kASubscriber;

  private static final double EPSILON = 1e-6;

  /** Record that contains all gains for the PIDF, and if they have changed */
  public record TunablePIDFGains(
      boolean hasChanged,
      double kP,
      double kI,
      double kD,
      double kS,
      double kG,
      double kV,
      double kA) {}

  private TunablePIDFGains lastGains = new TunablePIDFGains(false, 0, 0, 0, 0, 0, 0, 0);

  /**
   * Constructs a set of DoubleSubscribers for all PID and Feedforward gains
   *
   * @param prefix the prefix to be used for gains storing in NT
   */
  public TunablePIDF(String prefix) {
    kPSubscriber = DogLog.tunable(prefix + "/kP", 0.0);
    kISubscriber = DogLog.tunable(prefix + "/kI", 0.0);
    kDSubscriber = DogLog.tunable(prefix + "/kD", 0.0);
    kSSubscriber = DogLog.tunable(prefix + "/kS", 0.0);
    kGSubscriber = DogLog.tunable(prefix + "/kG", 0.0);
    kVSubscriber = DogLog.tunable(prefix + "/kV", 0.0);
    kASubscriber = DogLog.tunable(prefix + "/kA", 0.0);
  }

  /**
   * Gets all of the tuned PIDF gains.
   *
   * @return record of gains and if they have changed
   */
  public TunablePIDFGains getGains() {

    double kP = kPSubscriber.get();
    double kI = kISubscriber.get();
    double kD = kDSubscriber.get();
    double kS = kSSubscriber.get();
    double kG = kGSubscriber.get();
    double kV = kVSubscriber.get();
    double kA = kASubscriber.get();

    boolean hasChanged =
        Math.abs(lastGains.kP() - kP) > EPSILON
            || Math.abs(lastGains.kI() - kI) > EPSILON
            || Math.abs(lastGains.kD() - kD) > EPSILON
            || Math.abs(lastGains.kS() - kS) > EPSILON
            || Math.abs(lastGains.kG() - kG) > EPSILON
            || Math.abs(lastGains.kV() - kV) > EPSILON
            || Math.abs(lastGains.kA() - kA) > EPSILON;

    TunablePIDFGains gains = new TunablePIDFGains(hasChanged, kP, kI, kD, kS, kG, kV, kA);
    lastGains = gains;

    return gains;
  }
}
