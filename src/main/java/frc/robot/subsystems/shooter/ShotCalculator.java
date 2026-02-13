package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

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

public class ShotCalculator {
    private static ShotCalculator instance;

    public record ShotParameters(
        boolean isValid,
        Rotation2d turretAngle,
        Rotation2d hoodAngle,
        AngularVelocity flywheelVelocity
    ) {}

    private static double maxDistance;
    private static double minDistance;

    private static final InterpolatingTreeMap<Double, Rotation2d> hoodMap = 
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

    private static final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();

    static {
        maxDistance = 0; //TODO: Fill me in
        minDistance = 0;

        hoodMap.put(0.0, Rotation2d.fromRadians(0));
        
        flywheelMap.put(0.0, 0.0);
    }

    public ShotParameters calculateParameters() {
        Pose2d robotPose = RobotContainer.getDrivetrain().getPose();

        Translation2d target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

        Transform2d robotToTurret = new Transform2d(TurretConstants.ROBOT_TO_TURRET.getX(), TurretConstants.ROBOT_TO_TURRET.getY(), Rotation2d.kZero);

        Pose2d turretPose = robotPose.transformBy(robotToTurret);

        double distanceToTarget = turretPose.getTranslation().getDistance(target);

        boolean isValid = distanceToTarget > maxDistance || distanceToTarget < minDistance;
        Rotation2d turretAngle = target.minus(turretPose.getTranslation()).getAngle();
        Rotation2d hoodAngle = hoodMap.get(distanceToTarget);
        AngularVelocity flywheelVelocity = RadiansPerSecond.of(flywheelMap.get(distanceToTarget));

        return new ShotParameters(isValid, turretAngle, hoodAngle, flywheelVelocity);
    }

    public static ShotCalculator getInstance() {
        if (instance == null) {
            instance = new ShotCalculator();
        }

        return instance;
    }
}
