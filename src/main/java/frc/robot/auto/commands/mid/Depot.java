package frc.robot.auto.commands.mid;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotStates;
import frc.robot.auto.BaseAuto;
import frc.robot.auto.Paths;

public class Depot extends BaseAuto {

  public Depot() {
    super(
        Paths.midShootToDepotPickup.getStartingHolonomicPose().orElseThrow(),
        RobotState.getInstance().setStateCommand(RobotStates.INTAKING),
        Commands.parallel(
            RobotState.getInstance().setStateCommand(RobotStates.SCORING_NO_AGITATION),
            Commands.sequence(
                AutoBuilder.followPath(Paths.midShootToDepotPickup),
                AutoBuilder.followPath(Paths.depotPickupToShoot))));
  }
}
