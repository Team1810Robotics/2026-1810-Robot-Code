package frc.robot.subsystems.intake.roller;

public class RollerConstants {
  public static final int ROLLER_MOTOR_ID = 11;

  public static final double kP = 0.0;
  public static final double kS = 0.1;
  public static final double kV = .0025;

  public static final double MAX_VELOCITY = 6000;

  public enum RollerState {
    INTAKE(-5000),
    AUTO_INTAKE(-5000),
    OUT(MAX_VELOCITY),
    STOP(0),
    SHOOTING(0);

    private double velocity;

    private RollerState(double velocity) {
      this.velocity = velocity;
    }

    public double getVelocity() {
      return velocity;
    }
  }
}
