package frc.robot.subsystems.vision;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {
	private String name;

	public Vision(String limelightName) {
		this.name = limelightName;
	}

	public void setRobotOrientation(double yaw) {
		LimelightHelpers.SetRobotOrientation(name, yaw, 0, 0, 0, 0, 0);
	}

	public void setRobotOrientation() {
		double yaw = RobotContainer.getDrivetrain().getState().Pose.getRotation().getDegrees();

		LimelightHelpers.SetRobotOrientation(name, yaw, 0, 0, 0, 0, 0);
	}

	public void setIMUMode(int mode) {
		LimelightHelpers.SetIMUMode(name, mode);
	}

	public void resetIMUHeading(double yaw) {
		setIMUMode(1);
		setRobotOrientation(yaw);
	}

	@Override
	public void periodic() {
		setRobotOrientation(RobotContainer.getDrivetrain().getState().Pose.getRotation().getDegrees());

		PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

		RobotContainer.getDrivetrain().addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);

		DogLog.log("Vision/" + name + "/Pose", poseEstimate.pose);
	}
}
