package frc.robot.auto.commands.right;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.auto.BaseAuto;
import frc.robot.auto.Paths;
import frc.robot.commands.ShootWithAgitate;
import frc.robot.subsystems.intake.IntakeStates;

public class RightDoublePickup extends BaseAuto {
  public RightDoublePickup() {
    super(
        Paths.rightShootToPickup.getStartingHolonomicPose().orElseThrow(),
        Commands.parallel(
            AutoBuilder.followPath(Paths.rightShootToPickup),
            Commands.waitTime(Seconds.of(1))
                .andThen(RobotState.getInstance().setIntakeState(IntakeStates.INTAKE))),
        AutoBuilder.followPath(Paths.rightPickupToShoot),
        new ShootWithAgitate().withTimeout(Seconds.of(5)),
        Commands.parallel(
            AutoBuilder.followPath(Paths.rightShootToPickup),
            Commands.waitTime(Seconds.of(1))
                .andThen(RobotState.getInstance().setIntakeState(IntakeStates.INTAKE))),
        AutoBuilder.followPath(Paths.rightPickupToShoot),
        new ShootWithAgitate().withTimeout(Seconds.of(5)));
  }
}
