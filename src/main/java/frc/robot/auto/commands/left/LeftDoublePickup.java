package frc.robot.auto.commands.left;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.BaseAuto;
import frc.robot.auto.Paths;
import frc.robot.state.RobotState;
import frc.robot.state.RobotState.RobotStates;
import frc.robot.subsystems.indexer.kicker.KickerConstants.KickerState;
import frc.robot.subsystems.indexer.spindexer.SpindexerConstants.SpindexerState;

public class LeftDoublePickup extends BaseAuto {
  public LeftDoublePickup() {
    super(
        Paths.leftShootToPickup.getStartingHolonomicPose().orElseThrow(),
        Commands.parallel(
            AutoBuilder.followPath(Paths.leftShootToPickup),
            Commands.waitTime(Seconds.of(.5))
                .andThen(RobotState.getInstance().setStateCommand(RobotStates.INTAKING))),
        Commands.race(
            AutoBuilder.followPath(Paths.leftPickupToShoot),
            RobotContainer.getKickerSubsystem().kickCommand(KickerState.OUT),
            RobotContainer.getSpindexerSubsystem().spinCommand(SpindexerState.OUT)),
        RobotState.getInstance().setStateCommand(RobotStates.SCORING_WITH_AGITATION),
        Commands.parallel(
            AutoBuilder.followPath(Paths.leftShootToLowPickup),
            Commands.parallel(
                    RobotContainer.getKickerSubsystem().kickCommand(KickerState.OUT),
                    RobotContainer.getSpindexerSubsystem().spinCommand(SpindexerState.OUT))
                .withTimeout(Seconds.of(1))
                .andThen(RobotState.getInstance().setStateCommand(RobotStates.INTAKING))),
        AutoBuilder.followPath(Paths.leftLowPickupToShoot),
        RobotState.getInstance().setStateCommand(RobotStates.SCORING_WITH_AGITATION));
  }
}
