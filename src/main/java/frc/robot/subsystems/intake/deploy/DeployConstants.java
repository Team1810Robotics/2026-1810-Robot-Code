package frc.robot.subsystems.intake.deploy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class DeployConstants {
  public static final int LEFT_DELPOY_MOTOR_ID = 9;
  public static final int RIGHT_DEPLOY_MOTOR_ID = 10;
  public static final int LEFT_ENCODER_ID = 0;

  public static double kP = 4;
  public static double kD = 0.25;

  public static double kS = 0.75;
  public static double kG = 0.461;

  public static double ENCODER_OFFSET = Units.degreesToRotations(14);

  public static double GEAR_RATIO = 3;

  public static final Translation3d robotToIntake = new Translation3d(0.203, -0.34, 0.23);

  public enum DeployState {
    DEPLOY(Rotation2d.fromDegrees(12.5)),
    RETRACT(Rotation2d.fromDegrees(115.0)),
    AGITATE(Rotation2d.fromDegrees(37.5));

    private Rotation2d position;

    private DeployState(Rotation2d position) {
      this.position = position;
    }

    public Rotation2d getPosition() {
      return position;
    }
  }
}
