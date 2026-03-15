package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.util.geometry.AllianceFlipUtil;

public abstract class BaseAuto extends SequentialCommandGroup {
  protected BaseAuto(Pose2d startPose, Command... commands) {
    addCommands(
        Commands.runOnce(
            () -> RobotContainer.getDrivetrain().resetPose(AllianceFlipUtil.apply(startPose))));

    addCommands(commands);
  }
}
