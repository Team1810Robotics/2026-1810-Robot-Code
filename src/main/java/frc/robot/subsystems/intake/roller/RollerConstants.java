package frc.robot.subsystems.intake.roller;

public class RollerConstants {
  public static final int ROLLER_MOTOR_ID = 11;

  public enum rollerState {
    INTAKE(.75),
    OUT(-1),
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
