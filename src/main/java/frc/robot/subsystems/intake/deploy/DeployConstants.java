package frc.robot.subsystems.intake.deploy;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class DeployConstants {
  public static final int LEFT_DELPOY_MOTOR_ID = 9;
  public static final int RIGHT_DEPLOY_MOTOR_ID = 10;
  public static final int LEFT_ENCODER_ID = 0;

  public static double kP = 4;
  public static double kD = 0.25;

  public static double kS = 0.75;
  public static double kG = 0.461;

  public static double ENCODER_OFFSET = Units.degreesToRotations(43);

  public static double GEAR_RATIO = 3;

  public enum deployState {
    DEPLOY(Rotation2d.fromDegrees(14.5)),
    RETRACT(Rotation2d.fromDegrees(115.0)),
    AGITATE(Rotation2d.fromDegrees(30));

    private Rotation2d position;

    private deployState(Rotation2d position) {
      this.position = position;
    }

    public Rotation2d getPosition() {
      return position;
    }
  }
}
