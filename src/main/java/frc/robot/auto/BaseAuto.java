package frc.robot.auto;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotStates;
import frc.robot.util.geometry.AllianceFlipUtil;

public abstract class BaseAuto extends SequentialCommandGroup {
  protected BaseAuto(Pose2d startPose, Command... commands) {
    addCommands(
        Commands.runOnce(
            () -> RobotContainer.getDrivetrain().resetPose(AllianceFlipUtil.apply(startPose))));

    addCommands(commands);
  }

  protected static Command shootSequence() {
    return shootSequence(Seconds.of(4));
  }

  protected static Command shootSequence(double seconds) {
    return RobotState.getInstance()
        .setStateCommand(RobotStates.SCORING_WITH_AGITATION)
        .andThen(Commands.waitSeconds(seconds))
        .andThen(RobotState.getInstance().setStateCommand(RobotStates.INTAKING));
  }

  protected static Command shootSequence(Time time) {
    return shootSequence(time.in(Seconds));
  }
}
