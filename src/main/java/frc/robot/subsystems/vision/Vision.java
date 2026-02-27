package frc.robot.subsystems.vision;

// DriverStation not used in this subsystem
import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;

/** Subsystem for handling vision processing with Limelight cameras. */
public class Vision extends SubsystemBase {
  private final String limelightName;

  private final CommandSwerveDrivetrain drivetrain;

  private final String logPrefix;

  private final AprilTagFieldLayout layout;

  /**
   * Constructor for VisionSubsystem.
   *
   * @param name The name of the Limelight camera to use (e.g., "front_ll" or "rear_ll").
   */
  public Vision(String name) {
    this.limelightName = name;
    this.drivetrain = RobotContainer.getDrivetrain();
    this.logPrefix = "Vision/" + this.limelightName;
    this.layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    LimelightHelpers.setPipelineIndex(limelightName, 0);
    LimelightHelpers.SetIMUAssistAlpha(limelightName, .005);
    LimelightHelpers.SetIMUMode(limelightName, 1);
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

  /**
   * Gets the IDs of all AprilTags currently detected by the Limelight.
   *
   * @return An int array of detected AprilTag IDs, empty if none detected.
   */
  public int[] getTargetIDs() {
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);
    int[] ids = new int[fiducials.length];
    for (int i = 0; i < fiducials.length; i++) {
      ids[i] = fiducials[i].id;
    }
    return ids;
  }

  public Pose3d[] getTargetPoses() {
    int[] ids = getTargetIDs();

    Pose3d[] poses = new Pose3d[ids.length];
    for (int i = 0; i < ids.length; i++) {
      poses[i] = layout.getTagPose(ids[i]).orElse(new Pose3d());
    }

    return poses;
  }

  /**
   * Checks if a specific AprilTag ID is currently detected.
   *
   * @param id The AprilTag ID to check for.
   * @return True if the tag is detected, false otherwise.
   */
  public boolean isTagDetected(int id) {
    for (int detectedId : getTargetIDs()) {
      if (detectedId == id) return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      LimelightHelpers.SetIMUMode(limelightName, 1); // Set IMU mode to 1 while bot is diabled
    } else {
      LimelightHelpers.SetIMUMode(
          limelightName, 4); // Set IMU mode to 4 while bot is enabled, check docs for more details
    }

    // Update bot orientation for MT2 pose estimation + LL imu fusing
    LimelightHelpers.SetRobotOrientation(
        limelightName,
        drivetrain.getState().Pose.getRotation().getDegrees(),
        drivetrain.getState().Speeds.omegaRadiansPerSecond,
        drivetrain.getPigeon2().getPitch().getValueAsDouble(),
        0,
        drivetrain.getPigeon2().getRoll().getValueAsDouble(),
        0);

    if (!targetValid()) {
      log();
      return;
    }

    PoseEstimate botPoseMT2 = getBotpose();

    // Add the vision measurement to the drivetrain's pose estimator with appropriate timestamp
    drivetrain.addVisionMeasurement(botPoseMT2.pose, botPoseMT2.timestampSeconds);

    log();
  }

  public void log() {
    DogLog.log(logPrefix + "/Target Valid", targetValid());
    DogLog.log(logPrefix + "/Targets", getTargetPoses());

    if (!targetValid()) {
      DogLog.log(logPrefix + "/BotPose", new Pose2d());
      return;
    }

    DogLog.log(logPrefix + "/BotPose", getBotpose().pose);
  }
}
