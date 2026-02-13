package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.ShotParameters;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;

public class Shoot extends Command {
  private final FlywheelSubsystem flywheelSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  private boolean hasSpunUp = false;

  public Shoot() {
    this.flywheelSubsystem = RobotContainer.getFlywheelSubsystem();
    this.hoodSubsystem = RobotContainer.getHoodSubsystem();
    this.indexerSubsystem = RobotContainer.getIndexerSubsystem();

    addRequirements(flywheelSubsystem, hoodSubsystem, indexerSubsystem);
  }

  @Override
  public void execute() {
    ShotParameters params = ShotCalculator.getInstance().calculateParameters();
    if (!params.isValid()) return;

    hoodSubsystem.setPosition(params.hoodAngle());
    flywheelSubsystem.setVelocity(params.flywheelVelocity());

    if (!hasSpunUp && flywheelSubsystem.atTargetVelocity()) {
      hasSpunUp = true;
    }

    if (hasSpunUp) {
      indexerSubsystem.run(1, 1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.stop();
    flywheelSubsystem.stop();
    indexerSubsystem.fullstop();
  }
}
