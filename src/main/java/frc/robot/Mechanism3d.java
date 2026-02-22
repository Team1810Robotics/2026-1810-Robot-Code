package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;

public class Mechanism3d {
    private Mechanism3d() {}

    private static Mechanism3d instance;

    public static Mechanism3d getInstance() {
        if (instance == null) {
            instance = new Mechanism3d();
        }
        return instance;
    }

    private TurretSubsystem turretSubsystem = RobotContainer.getTurretSubsystem();

    public void log() {
        Rotation2d turretAngle = turretSubsystem.getTurretAngle();

        DogLog.log("Mechanisms/Turret", new Pose3d(
            TurretConstants.ROBOT_TO_TURRET.getTranslation(),
            new Rotation3d(turretAngle)
        )
        );
    }
}
