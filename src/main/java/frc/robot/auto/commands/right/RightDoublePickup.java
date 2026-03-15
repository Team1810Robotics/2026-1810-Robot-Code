package frc.robot.auto.commands.right;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.BaseAuto;
import frc.robot.auto.Paths;
import frc.robot.commands.ShootWithAgitate;
import frc.robot.state.RobotState;
import frc.robot.subsystems.indexer.kicker.KickerConstants.KickerState;
import frc.robot.subsystems.indexer.spindexer.SpindexerConstants.SpindexerState;
import frc.robot.subsystems.intake.IntakeStates;

public class RightDoublePickup extends BaseAuto {
  public RightDoublePickup() {
    super(
        Paths.rightShootToPickup.getStartingHolonomicPose().orElseThrow(),
        Commands.parallel(
            AutoBuilder.followPath(Paths.rightShootToPickup),
            Commands.waitTime(Seconds.of(.5))
                .andThen(RobotState.getInstance().setIntakeState(IntakeStates.INTAKE))),
        Commands.race(
            AutoBuilder.followPath(Paths.rightPickupToShoot),
            RobotContainer.getKickerSubsystem().kickCommand(KickerState.OUT),
            RobotContainer.getSpindexerSubsystem().spinCommand(SpindexerState.OUT)),
        new ShootWithAgitate().withTimeout(Seconds.of(4)),
        Commands.parallel(
            AutoBuilder.followPath(Paths.rightShootToLowPickup),
            Commands.parallel(
                    RobotContainer.getKickerSubsystem().kickCommand(KickerState.OUT),
                    RobotContainer.getSpindexerSubsystem().spinCommand(SpindexerState.OUT))
                .withTimeout(Seconds.of(1))
                .andThen(RobotState.getInstance().setIntakeState(IntakeStates.INTAKE))),
        AutoBuilder.followPath(Paths.rightLowPickupToShoot),
        new ShootWithAgitate().withTimeout(Seconds.of(5)));
  }
}
