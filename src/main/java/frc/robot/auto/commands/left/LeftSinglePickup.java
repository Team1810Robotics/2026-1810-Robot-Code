package frc.robot.auto.commands.left;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.BaseAuto;
import frc.robot.auto.Paths;
import frc.robot.commands.ShootWithAgitate;
import frc.robot.state.RobotState;
import frc.robot.state.RobotState.RobotStates;

public class LeftSinglePickup extends BaseAuto {

  public LeftSinglePickup() {
    super(
        Paths.leftShootToPickup.getStartingHolonomicPose().orElseThrow(),
        Commands.parallel(
            AutoBuilder.followPath(Paths.leftShootToPickup),
            Commands.waitTime(Seconds.of(1))
                .andThen(RobotState.getInstance().setStateCommand(RobotStates.INTAKING))),
        AutoBuilder.followPath(Paths.rightPickupToShoot),
        new ShootWithAgitate().withTimeout(Seconds.of(5)));
  }
}
