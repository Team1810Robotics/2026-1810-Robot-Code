package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.Region;

public class ShotCalculator {
  private static ShotCalculator instance;

  public record ShotParameters(
      boolean isValid,
      Rotation2d turretAngle,
      Rotation2d hoodAngle,
      AngularVelocity flywheelVelocity) {}

  private static double maxDistance;
  private static double minDistance;

  private static final InterpolatingTreeMap<Double, Rotation2d> hoodMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

  private static final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();

  static {
    maxDistance = 0; // TODO: Fill me in
    minDistance = 0;

    hoodMap.put(0.0, Rotation2d.fromRadians(0));

    flywheelMap.put(0.0, 0.0);
  }

  public ShotParameters calculateParameters() {
    Pose2d robotPose = RobotContainer.getDrivetrain().getPose();
    Optional<Region> robotRegion =
        Region.getRegion(robotPose);

    if (robotRegion.isEmpty()) {
      return new ShotParameters(false, Rotation2d.kZero, Rotation2d.kZero, RadiansPerSecond.of(0));
    }

    Translation2d target;

    switch (robotRegion.get()) {
      case BLUE_ALLIANCE_ZONE:
        target = FieldConstants.Hub.topCenterPoint.toTranslation2d();
        break;
      case RED_ALLIANCE_ZONE:
        target = FieldConstants.Hub.topCenterPoint.toTranslation2d();
        break;
      case UPPER_NEUTRAL_ZONE:
        target =
            new Translation2d(
                FieldConstants.LinesVertical.allianceZone / 2, FieldConstants.fieldWidth * (2.0 / 3.0));
        break;
      case LOWER_NEUTRAL_ZONE:
        target =
            new Translation2d(
                FieldConstants.LinesVertical.allianceZone / 2, FieldConstants.fieldWidth * (1.0 / 3.0));
        break;
      default:
        target = new Translation2d();
        break;
    }

    target = AllianceFlipUtil.apply(target);

    Transform2d robotToTurret =
        new Transform2d(
            TurretConstants.ROBOT_TO_TURRET.getX(),
            TurretConstants.ROBOT_TO_TURRET.getY(),
            Rotation2d.kZero);

    Pose2d turretPose = robotPose.transformBy(robotToTurret);

    double distanceToTarget = turretPose.getTranslation().getDistance(target);

    boolean isValid = distanceToTarget > maxDistance || distanceToTarget < minDistance;
    Rotation2d turretAngle = target.minus(turretPose.getTranslation()).getAngle();
    Rotation2d hoodAngle = hoodMap.get(distanceToTarget);
    AngularVelocity flywheelVelocity = RadiansPerSecond.of(flywheelMap.get(distanceToTarget));

    DogLog.log("ShotCalculator/DistanceToTarget", distanceToTarget);
    DogLog.log("ShotCalculator/Target", new Pose2d(target, Rotation2d.kZero));
    DogLog.log("ShotCalculator/Region", robotRegion.get().name());

    return new ShotParameters(isValid, turretAngle, hoodAngle, flywheelVelocity);
  }

  public static ShotCalculator getInstance() {
    if (instance == null) {
      instance = new ShotCalculator();
    }

    return instance;
  }
}
