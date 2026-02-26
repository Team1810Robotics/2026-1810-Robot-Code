package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class IntakeConstants {
  public static int LEFT_DELPOY_MOTOR_ID = 9;
  public static int RIGHT_DEPLOY_MOTOR_ID = 10;
  public static int LEFT_ENCODER_ID = 0;
  public static int ROLLER_MOTOR_ID = 11;

  public static double kP = 2;
  public static double kD = 0;

  public static double kS = 0.8;
  public static double kG = 0;

  public static double ENCODER_OFFSET = Units.degreesToRotations(43);

  public static double GEAR_RATIO = 3;

  public enum rollerState {
    INTAKE(1),
    OUT(-1),
    STOP(0);

    private double power;

    private rollerState(double power) {
      this.power = power;
    }

    public double getPower() {
      return power;
    }
  }

  public enum deployState {
    DEPLOY(Degrees.of(13.5)),
    RETRACT(Degrees.of(120.0));

    private Angle position;

    private deployState(Angle position) {
      this.position = position;
    }

    public double getPosition() {
      return position.in(Radians);
    }
  }

  public enum IntakeState {
    OUT_INTAKE(rollerState.INTAKE, deployState.DEPLOY),
    OUT_STOP(rollerState.STOP, deployState.DEPLOY),
    RETRACT(rollerState.STOP, deployState.RETRACT);

    private final rollerState roller;
    private final deployState deploy;

    private IntakeState(rollerState roller, deployState deploy) {
      this.roller = roller;
      this.deploy = deploy;
    }

    public rollerState getRollerState() {
      return this.roller;
    }

    public deployState getDeployState() {
      return this.deploy;
    }
  }
}

// I'm special.
