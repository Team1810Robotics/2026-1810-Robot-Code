package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.util.field.Region;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.Optional;

public class ShotCalculator {
  private static ShotCalculator instance;

  public record ShotParameters(
      boolean isValid,
      Rotation2d turretAngle,
      Rotation2d hoodAngle,
      AngularVelocity flywheelVelocity) {}

  private static double maxDistance;
  private static double minDistance;
  private static double phaseDelay;

  private static final InterpolatingTreeMap<Double, Rotation2d> scoringHoodMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

  private static final InterpolatingDoubleTreeMap scoringFlywheelMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap scoringTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingTreeMap<Double, Rotation2d> passingHoodMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

  private static final InterpolatingDoubleTreeMap passingFlywheelMap =
      new InterpolatingDoubleTreeMap();

  private ShotParameters latestScoringParams;
  private ShotParameters latestPassingParams;

  static {
    maxDistance = 5.5;
    minDistance = 1;
    phaseDelay = 0.03;

    scoringHoodMap.put(1.48, Rotation2d.fromDegrees(0));
    scoringHoodMap.put(1.94, Rotation2d.fromDegrees(1));
    scoringHoodMap.put(2.54, Rotation2d.fromDegrees(2));
    scoringHoodMap.put(3.0, Rotation2d.fromDegrees(3.5));
    scoringHoodMap.put(3.78, Rotation2d.fromDegrees(5));
    scoringHoodMap.put(4.2, Rotation2d.fromDegrees(7));
    scoringHoodMap.put(5.1, Rotation2d.fromDegrees(8.5));

    scoringFlywheelMap.put(1.48, 29.0);
    scoringFlywheelMap.put(1.94, 31.5);
    scoringFlywheelMap.put(2.52, 34.5);
    scoringFlywheelMap.put(3.0, 37.0);
    scoringFlywheelMap.put(3.78, 40.0);
    scoringFlywheelMap.put(4.2, 41.5);
    scoringFlywheelMap.put(5.1, 44.67);

    scoringTimeOfFlightMap.put(1.45, 1.083);
    scoringTimeOfFlightMap.put(2.62, 1.167);
    scoringTimeOfFlightMap.put(3.23, 1.217);
    scoringTimeOfFlightMap.put(4.03, 1.3);
    scoringTimeOfFlightMap.put(5.39, 1.433);

    passingHoodMap.put(3.8, Rotation2d.fromDegrees(5));
    passingHoodMap.put(5.0, Rotation2d.fromDegrees(7));
    passingHoodMap.put(6.7, Rotation2d.fromDegrees(12));

    passingFlywheelMap.put(3.8, 40.0);
    passingFlywheelMap.put(5.0, 45.0);
    passingFlywheelMap.put(6.7, 55.0);
  }

  public ShotParameters calculateScoringParameters() {

    if (latestScoringParams != null) {
      return latestScoringParams;
    }

    Pose2d robotPose = RobotContainer.getDrivetrain().getPose();
    ChassisSpeeds robotRelativeVelocity = RobotContainer.getDrivetrain().getRobotRelativeSpeeds();

    robotPose =
        robotPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    Optional<Region> robotRegion = Region.getRegion(robotPose);

    ChassisSpeeds robotVelocity = RobotContainer.getDrivetrain().getFieldRelativeSpeeds();
    double robotAngle = robotPose.getRotation().getRadians();

    if (robotRegion.isEmpty()) {
      return new ShotParameters(false, Rotation2d.kZero, Rotation2d.kZero, RadiansPerSecond.of(0));
    }

    Translation2d realTarget = AllianceFlipUtil.apply(ShotConstants.hub);

    Translation2d virtualTarget;

    Transform2d robotToTurret =
        new Transform2d(
            TurretConstants.ROBOT_TO_TURRET.getX(),
            TurretConstants.ROBOT_TO_TURRET.getY(),
            Rotation2d.kZero);

    Pose2d turretPose = robotPose.transformBy(robotToTurret);

    double distanceToRealTarget = turretPose.getTranslation().getDistance(realTarget);

    boolean isValid = true;
    Rotation2d hoodAngle, turretAngle;
    AngularVelocity flywheelVelocity;

    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getY() * Math.cos(robotAngle)
                    - robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getX() * Math.cos(robotAngle)
                    - robotToTurret.getY() * Math.sin(robotAngle));

    double tof = scoringTimeOfFlightMap.get(distanceToRealTarget);

    Translation2d velocityCompensation =
        new Translation2d(-(turretVelocityX * tof), -(turretVelocityY * tof));

    virtualTarget = realTarget.plus(velocityCompensation);

    double distanceToVirtualTarget = turretPose.getTranslation().getDistance(virtualTarget);

    turretAngle = virtualTarget.minus(turretPose.getTranslation()).getAngle();

    hoodAngle = scoringHoodMap.get(distanceToVirtualTarget);
    flywheelVelocity = RotationsPerSecond.of(scoringFlywheelMap.get(distanceToVirtualTarget));

    DogLog.log("ShotCalculator/Distance To Target", distanceToRealTarget, Meters);
    DogLog.log("ShotCalculator/Real Target", new Pose2d(realTarget, Rotation2d.kZero));
    DogLog.log("ShotCalculator/Virtual Target", new Pose2d(virtualTarget, Rotation2d.kZero));

    DogLog.log("ShotCalculator/Parameters/Is Valid", isValid);
    DogLog.log("ShotCalculator/Parameters/Turret Angle", turretAngle.getDegrees(), Degrees);
    DogLog.log("ShotCalculator/Parameters/Hood Angle", hoodAngle.getDegrees(), Degrees);
    DogLog.log(
        "ShotCalculator/Parameters/Flywheel Velocity", flywheelVelocity.in(RotationsPerSecond));

    latestScoringParams = new ShotParameters(isValid, turretAngle, hoodAngle, flywheelVelocity);

    return latestScoringParams;
  }

  public ShotParameters calculatePassingParameters() {

    if (latestPassingParams != null) {
      return latestPassingParams;
    }

    Pose2d robotPose = RobotContainer.getDrivetrain().getPose();

    Optional<Region> robotRegion = Region.getRegion(robotPose);

    if (robotRegion.isEmpty()) {
      return new ShotParameters(false, Rotation2d.kZero, Rotation2d.kZero, RadiansPerSecond.of(0));
    }

    Translation2d target;

    if (robotRegion.get() == Region.UPPER_NEUTRAL_ZONE) {
      target = ShotConstants.upperPass;
    } else if (robotRegion.get() == Region.LOWER_NEUTRAL_ZONE) {
      target = ShotConstants.lowerPass;
    } else {
      return new ShotParameters(false, Rotation2d.kZero, Rotation2d.kZero, RadiansPerSecond.of(0));
    }

    target = AllianceFlipUtil.apply(target);

    Transform2d robotToTurret =
        new Transform2d(
            TurretConstants.ROBOT_TO_TURRET.getX(),
            TurretConstants.ROBOT_TO_TURRET.getY(),
            Rotation2d.kZero);

    Pose2d turretPose = robotPose.transformBy(robotToTurret);

    double distanceToTarget = turretPose.getTranslation().getDistance(target);

    boolean isValid;
    Rotation2d hoodAngle, turretAngle;
    AngularVelocity flywheelVelocity;

    double distanceToVirtualTarget = turretPose.getTranslation().getDistance(target);

    turretAngle = target.minus(turretPose.getTranslation()).getAngle();

    Rotation2d turretAngleTurretFrame =
        RobotContainer.getTurretSubsystem().robotRelativeToTurret(turretAngle);

    if (turretAngleTurretFrame.getDegrees() > TurretConstants.MAX_ANGLE.in(Degrees)
        || turretAngleTurretFrame.getDegrees() < TurretConstants.MIN_ANGLE.in(Degrees)) {
      isValid = false;
    } else {
      isValid = true;
    }

    hoodAngle = scoringHoodMap.get(distanceToVirtualTarget);
    flywheelVelocity = RotationsPerSecond.of(scoringFlywheelMap.get(distanceToVirtualTarget));

    DogLog.log("ShotCalculator/Distance To Target", distanceToTarget, Meters);
    DogLog.log("ShotCalculator/Real Target", new Pose2d(target, Rotation2d.kZero));
    DogLog.log("ShotCalculator/Virtual Target", new Pose2d());

    DogLog.log("ShotCalculator/Parameters/Is Valid", isValid);
    DogLog.log("ShotCalculator/Parameters/Turret Angle", turretAngle.getDegrees(), Degrees);
    DogLog.log("ShotCalculator/Parameters/Hood Angle", hoodAngle.getDegrees(), Degrees);
    DogLog.log(
        "ShotCalculator/Parameters/Flywheel Velocity", flywheelVelocity.in(RotationsPerSecond));

    latestPassingParams = new ShotParameters(isValid, turretAngle, hoodAngle, flywheelVelocity);

    return latestPassingParams;
  }

  public Translation2d getTarget() {
    Pose2d robotPose = RobotContainer.getDrivetrain().getPose();
    Optional<Region> robotRegion = Region.getRegion(robotPose);

    if (robotRegion.isEmpty()) {
      return new Translation2d();
    }

    Translation2d realTarget;

    switch (robotRegion.get()) {
      case BLUE_ALLIANCE_ZONE:
        realTarget = ShotConstants.hub;
        break;
      case RED_ALLIANCE_ZONE:
        realTarget = ShotConstants.hub;
        break;
      case UPPER_NEUTRAL_ZONE:
        realTarget = ShotConstants.upperPass;
        break;
      case LOWER_NEUTRAL_ZONE:
        realTarget = ShotConstants.lowerPass;
        break;
      default:
        realTarget = new Translation2d();
        break;
    }

    realTarget = AllianceFlipUtil.apply(realTarget);

    return realTarget;
  }

  public Rotation2d getTurretAngleParameter(Translation2d target) {
    Pose2d robotPose = RobotContainer.getDrivetrain().getPose();

    Transform2d robotToTurret =
        new Transform2d(
            TurretConstants.ROBOT_TO_TURRET.getX(),
            TurretConstants.ROBOT_TO_TURRET.getY(),
            Rotation2d.kZero);

    Pose2d turretPose = robotPose.transformBy(robotToTurret);

    return target.minus(turretPose.getTranslation()).getAngle();
  }

  public Rotation2d getTurretAngleParameter() {
    Translation2d target = getTarget();
    Pose2d robotPose = RobotContainer.getDrivetrain().getPose();

    Transform2d robotToTurret =
        new Transform2d(
            TurretConstants.ROBOT_TO_TURRET.getX(),
            TurretConstants.ROBOT_TO_TURRET.getY(),
            Rotation2d.kZero);

    Pose2d turretPose = robotPose.transformBy(robotToTurret);

    return target.minus(turretPose.getTranslation()).getAngle();
  }

  public void clearParams() {
    latestScoringParams = null;
    latestPassingParams = null;
  }

  public static ShotCalculator getInstance() {
    if (instance == null) {
      instance = new ShotCalculator();
    }

    return instance;
  }
}
