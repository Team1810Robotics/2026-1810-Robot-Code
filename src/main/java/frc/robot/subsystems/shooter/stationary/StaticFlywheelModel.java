package frc.robot.subsystems.shooter.stationary.flywheel;

/** A measured shot-speed fit. NaN means the model cannot be used for that input. */
public record FlywheelModel(
    double a,
    double b,
    double c,
    double minX,
    double maxX,
    double maxRps,
    boolean calibrated) {

  public double rpsFor(double x) {
    if (!calibrated
        || !Double.isFinite(x)
        || !Double.isFinite(a)
        || !Double.isFinite(b)
        || !Double.isFinite(c)
        || !Double.isFinite(minX)
        || !Double.isFinite(maxX)
        || !Double.isFinite(maxRps)
        || minX >= maxX
        || maxRps <= 0.0
        || x < minX
        || x > maxX) {
      return Double.NaN;
    }

    double rps = (a * x + b) * x + c;
    // Reject invalid speeds rather than silently changing a bad fit by clamping it.
    return Double.isFinite(rps) && rps > 0.0 && rps <= maxRps ? rps : Double.NaN;
  }
}
