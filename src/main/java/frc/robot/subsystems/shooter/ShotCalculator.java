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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShotConstants.ShootingModes;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.util.field.FieldConstants;
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

  private ShotParameters latestParams;

  static {
    maxDistance = 5.5;
    minDistance = 1;
    phaseDelay = 0.03;

    scoringHoodMap.put(1.491, Rotation2d.fromDegrees(0)); // -.5
    scoringHoodMap.put(2.091, Rotation2d.fromDegrees(5));
    scoringHoodMap.put(2.48, Rotation2d.fromDegrees(6));
    scoringHoodMap.put(3.02, Rotation2d.fromDegrees(7));
    scoringHoodMap.put(3.39, Rotation2d.fromDegrees(8.5));
    scoringHoodMap.put(3.5, Rotation2d.fromDegrees(9.5));
    scoringHoodMap.put(4.11, Rotation2d.fromDegrees(11.5));
    scoringHoodMap.put(4.4, Rotation2d.fromDegrees(12.5));
    scoringHoodMap.put(4.435, Rotation2d.fromDegrees(12.75));
    scoringHoodMap.put(4.683, Rotation2d.fromDegrees(14.5));
    scoringHoodMap.put(5.05, Rotation2d.fromDegrees(17));
    scoringHoodMap.put(5.606, Rotation2d.fromDegrees(19));
    scoringHoodMap.put(6.032, Rotation2d.fromDegrees(20));

    scoringFlywheelMap.put(1.491, 28.50); // -1.25
    scoringFlywheelMap.put(2.091, 31.75);
    scoringFlywheelMap.put(2.48, 33.25);
    scoringFlywheelMap.put(3.02, 34.75);
    scoringFlywheelMap.put(3.39, 36.25);
    scoringFlywheelMap.put(3.5, 36.75);
    scoringFlywheelMap.put(4.11, 38.0);
    scoringFlywheelMap.put(4.4, 38.5);
    scoringFlywheelMap.put(4.683, 39.75);
    scoringFlywheelMap.put(5.05, 40.75);
    scoringFlywheelMap.put(5.606, 41.5);
    scoringFlywheelMap.put(6.032, 43.0);

    scoringTimeOfFlightMap.put(1.658, 0.917);
    scoringTimeOfFlightMap.put(2.092, 0.967);
    scoringTimeOfFlightMap.put(2.467, 1.117);
    scoringTimeOfFlightMap.put(2.96, 1.167);
    scoringTimeOfFlightMap.put(3.4, 1.223);
    scoringTimeOfFlightMap.put(4.106, 1.267);
    scoringTimeOfFlightMap.put(4.57, 1.167); // vro idk (i do)
    scoringTimeOfFlightMap.put(5.05, 1.183);
    scoringTimeOfFlightMap.put(5.5, 1.133);
    scoringTimeOfFlightMap.put(6.1, 1.133);

    passingHoodMap.put(3.8, Rotation2d.fromDegrees(5));
    passingHoodMap.put(5.0, Rotation2d.fromDegrees(7)); // , 3.4, , 2.96,
    passingHoodMap.put(6.7, Rotation2d.fromDegrees(12));

    passingFlywheelMap.put(3.8, 40.0);
    passingFlywheelMap.put(5.0, 45.0);
    passingFlywheelMap.put(6.7, 55.0);
  }

  public ShotParameters calculateParameters() {
    if (latestParams != null) {
      return latestParams;
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

    Translation2d realTarget;

    ShootingModes mode;

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    if (Region.isOurAllianceZone(robotRegion.get())) {
      mode = ShootingModes.SCORING;
      realTarget = AllianceFlipUtil.apply(ShotConstants.hub);
    } else if (Region.inOpposingAllianceZone(robotRegion.get())) {
      mode = ShootingModes.PASSING;
      if (robotPose.getTranslation().getY() > FieldConstants.fieldWidth / 2.0) {
        realTarget =
            alliance == Alliance.Blue ? ShotConstants.blueUpperPass : ShotConstants.redUpperPass;
      } else {
        realTarget =
            alliance == Alliance.Blue ? ShotConstants.blueLowerPass : ShotConstants.redLowerPass;
      }
    } else if (robotRegion.get() == Region.UPPER_NEUTRAL_ZONE) {
      mode = ShootingModes.PASSING;
      realTarget =
          alliance == Alliance.Blue ? ShotConstants.blueUpperPass : ShotConstants.redUpperPass;
    } else if (robotRegion.get() == Region.LOWER_NEUTRAL_ZONE) {
      mode = ShootingModes.PASSING;
      realTarget =
          alliance == Alliance.Blue ? ShotConstants.blueLowerPass : ShotConstants.redLowerPass;
    } else {
      return new ShotParameters(false, Rotation2d.kZero, Rotation2d.kZero, RadiansPerSecond.of(0));
    }

    Translation2d virtualTarget;

    Transform2d robotToTurret =
        new Transform2d(
            TurretConstants.ROBOT_TO_TURRET.getX(),
            TurretConstants.ROBOT_TO_TURRET.getY(),
            Rotation2d.kZero);

    Pose2d turretPose = robotPose.transformBy(robotToTurret);

    double distanceToRealTarget = turretPose.getTranslation().getDistance(realTarget);

    boolean isValid;
    Rotation2d hoodAngle, turretAngle;
    AngularVelocity flywheelVelocity;

    if (mode == ShootingModes.SCORING) {
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

      Rotation2d turretAngleTurretFrame =
          RobotContainer.getTurretSubsystem().fieldToMotor(turretAngle);

      if (turretAngleTurretFrame.getDegrees() > TurretConstants.MAX_ANGLE.in(Degrees)
          || turretAngleTurretFrame.getDegrees()
              < TurretConstants.MIN_ANGLE.in(Degrees)) { // I AM BAD DELETE ME LATER
        isValid = false;
      } else {
        isValid = true;
      }

      hoodAngle = scoringHoodMap.get(distanceToVirtualTarget);
      flywheelVelocity = RotationsPerSecond.of(scoringFlywheelMap.get(distanceToVirtualTarget));
    } else {
      virtualTarget = realTarget;

      double distanceToVirtualTarget = turretPose.getTranslation().getDistance(virtualTarget);

      turretAngle = virtualTarget.minus(turretPose.getTranslation()).getAngle();

      Rotation2d turretAngleTurretFrame =
          RobotContainer.getTurretSubsystem().fieldToMotor(turretAngle);

      if (turretAngleTurretFrame.getDegrees() > TurretConstants.MAX_ANGLE.in(Degrees)
          || turretAngleTurretFrame.getDegrees()
              < TurretConstants.MIN_ANGLE.in(Degrees)) { // I AM BAD DELETE ME LATER
        isValid = false;
      } else {
        isValid = true;
      }

      hoodAngle = scoringHoodMap.get(distanceToVirtualTarget);
      flywheelVelocity = RotationsPerSecond.of(scoringFlywheelMap.get(distanceToVirtualTarget));
    }

    DogLog.log("ShotCalculator/Distance To Target", distanceToRealTarget, Meters);
    DogLog.log("ShotCalculator/Real Target", new Pose2d(realTarget, Rotation2d.kZero));
    DogLog.log("ShotCalculator/Virtual Target", new Pose2d(virtualTarget, Rotation2d.kZero));

    DogLog.log("ShotCalculator/Parameters/Is Valid", isValid);
    DogLog.log("ShotCalculator/Parameters/Turret Angle", turretAngle.getDegrees(), Degrees);
    DogLog.log("ShotCalculator/Parameters/Hood Angle", hoodAngle.getDegrees(), Degrees);
    DogLog.log(
        "ShotCalculator/Parameters/Flywheel Velocity", flywheelVelocity.in(RotationsPerSecond));
    DogLog.log("ShotCalculator/Mode", mode.toString());

    latestParams = new ShotParameters(isValid, turretAngle, hoodAngle, flywheelVelocity);

    return latestParams;
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
        realTarget = ShotConstants.blueUpperPass;
        break;
      case LOWER_NEUTRAL_ZONE:
        realTarget = ShotConstants.blueLowerPass;
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
    latestParams = null;
  }

  public static ShotCalculator getInstance() {
    if (instance == null) {
      instance = new ShotCalculator();
    }

    return instance;
  }
}
