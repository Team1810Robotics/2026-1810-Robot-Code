package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
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
  private IntakeSubsystem intakeSubsystem = RobotContainer.getIntakeSubsystem();
  private HoodSubsystem hoodSubsystem = RobotContainer.getHoodSubsystem();

  public void log() {
    Rotation2d turretAngle = turretSubsystem.getTurretAngle();
    Rotation2d hoodAngle = hoodSubsystem.getMotorPosition();
    Rotation2d intakeAngle = intakeSubsystem.getPosition();

    Pose3d turretPose = new Pose3d(TurretConstants.ROBOT_TO_TURRET.getTranslation(), new Rotation3d(turretAngle));
    Pose3d intakePose = new Pose3d(IntakeConstants.robotToIntake, new Rotation3d(0, -intakeAngle.getRadians(), 0));
    Pose3d hoodPose = turretPose.transformBy(new Transform3d(HoodConstants.turretToHood, new Rotation3d(0,hoodAngle.getRadians(),0)));

    DogLog.log(
        "Mechanisms/Turret",
        turretPose);
    DogLog.log("Mehanisms/Intake", 
        intakePose);
    DogLog.log("Mechanisms/Hood", hoodPose);
  }
}
