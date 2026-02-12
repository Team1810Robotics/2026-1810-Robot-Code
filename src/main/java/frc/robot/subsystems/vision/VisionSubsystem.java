package frc.robot.subsystems.vision;

// DriverStation not used in this subsystem
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

/** Subsystem for handling vision processing with Limelight cameras. */
public class VisionSubsystem extends SubsystemBase {
  private final String limelightName;

  private final CommandSwerveDrivetrain drivetrain;

  /**
   * Constructor for VisionSubsystem.
   *
   * @param name The name of the Limelight camera to use (e.g., "front_ll" or "rear_ll").
   */
  public VisionSubsystem(String name) {
    this.limelightName = name;
    this.drivetrain = RobotContainer.getDrivetrain();

    LimelightHelpers.setPipelineIndex(limelightName, 0);
    LimelightHelpers.SetIMUAssistAlpha(limelightName, .001);
  }

  public int getTargetID() {
    return (int) LimelightHelpers.getFiducialID(limelightName);
  }

  /* Checks if the Limelight has a valid target. */
  public boolean targetValid() {
    return LimelightHelpers.getTV(limelightName);
  }

  /**
   * Gets the robot's pose estimate from the Limelight using the MegaTag2 pipeline.
   *
   * @return A PoseEstimate object containing the robot's estimated pose and the timestamp of the
   *     measurement.
   */
  public PoseEstimate getBotpose() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      LimelightHelpers.SetIMUMode(limelightName, 1); // Set IMU mode to 1 while bot is diabled
    } else {
      LimelightHelpers.SetIMUMode(
          limelightName, 4); // Set IMU mode to 4 while bot is enabled, check docs for more details
    }

    // Update bot orientation for MT2 pose estimation
    LimelightHelpers.SetRobotOrientation(
        limelightName,
        drivetrain.getState().Pose.getRotation().getDegrees(),
        drivetrain.getState().Speeds.omegaRadiansPerSecond,
        0,
        0,
        0,
        0);

    if (!targetValid()) {
      DogLog.log("Vision/BotPose", new Pose2d());

      return;
    }

    PoseEstimate botPoseMT2 = getBotpose();

    // Add the vision measurement to the drivetrain's pose estimator with appropriate timestamp
    drivetrain.addVisionMeasurement(botPoseMT2.pose, botPoseMT2.timestampSeconds);

    DogLog.log("Vision/BotPose", getBotpose().pose);
  }
}
