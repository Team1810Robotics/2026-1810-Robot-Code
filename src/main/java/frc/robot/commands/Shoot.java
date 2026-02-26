package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.indexer.IndexerConstants.indexerState;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeConstants.rollerState;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;

public class Shoot extends Command {
  private final FlywheelSubsystem flywheelSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private boolean hasSpunUp = false;

  public Shoot() {
    this.flywheelSubsystem = RobotContainer.getFlywheelSubsystem();
    this.hoodSubsystem = RobotContainer.getHoodSubsystem();
    this.indexerSubsystem = RobotContainer.getIndexerSubsystem();
    this.intakeSubsystem = RobotContainer.getIntakeSubsystem();

    addRequirements(flywheelSubsystem, hoodSubsystem, indexerSubsystem, intakeSubsystem);
  }

  @Override
  public void execute() {
    // ShotParameters params = ShotCalculator.getInstance().calculateParameters();
    // if (!params.isValid()) return;

    // hoodSubsystem.setPosition(Rotation2d.fromDegrees(20));
    flywheelSubsystem.setVelocity(RotationsPerSecond.of(50));

    if (!hasSpunUp && flywheelSubsystem.atTargetVelocity()) {
      hasSpunUp = true;
    }

    if (hasSpunUp) {
      indexerSubsystem.index(indexerState.IN);
      intakeSubsystem.roller(rollerState.INTAKE);
    }
  }

  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.stop();
    flywheelSubsystem.stop();
    indexerSubsystem.fullStop();
    intakeSubsystem.roller(rollerState.STOP);
  }
}
