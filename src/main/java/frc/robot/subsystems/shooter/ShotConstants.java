package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.util.field.FieldConstants;

public class ShotConstants {
  public static final Transform2d robotToTurret =
      new Transform2d(
          TurretConstants.ROBOT_TO_TURRET.getX(),
          TurretConstants.ROBOT_TO_TURRET.getY(),
          Rotation2d.kZero);

  public static final Translation2d hub = FieldConstants.Hub.topCenterPoint.toTranslation2d();

  public static final Translation2d blueUpperPass =
      new Translation2d(
          FieldConstants.LinesVertical.allianceZone / 2, FieldConstants.fieldWidth * (3.0 / 4.0));

  public static final Translation2d blueLowerPass =
      new Translation2d(
          FieldConstants.LinesVertical.allianceZone / 2, FieldConstants.fieldWidth * (1.0 / 4.0));

  public static final Translation2d redUpperPass =
      new Translation2d(
          FieldConstants.LinesVertical.oppAllianceZone
              + ((FieldConstants.fieldLength - FieldConstants.LinesVertical.oppAllianceZone) / 2.0),
          FieldConstants.fieldWidth * (3.0 / 4.0));
  public static final Translation2d redLowerPass =
      new Translation2d(
          FieldConstants.LinesVertical.oppAllianceZone
              + ((FieldConstants.fieldLength - FieldConstants.LinesVertical.oppAllianceZone) / 2.0),
          FieldConstants.fieldWidth * (1.0 / 4.0));

  public enum ShootingModes {
    PASSING,
    SCORING
  }
}
