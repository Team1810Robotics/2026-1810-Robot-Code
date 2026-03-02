package frc.robot.subsystems.intake.roller;

public class RollerConstants {
  public static final int ROLLER_MOTOR_ID = 11;

  public static final double kP = 0.0;
  public static final double kS = 0.1;
  public static final double kV = .0025;

  public enum rollerState {
    INTAKE(-3000),
    OUT(3000),
    STOP(0);

    private double velocity;

    private rollerState(double velocity) {
      this.velocity = velocity;
    }

    public double getVelocity() {
      return velocity;
    }
  }
}
