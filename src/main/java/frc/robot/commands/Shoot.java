package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotStates;
import frc.robot.subsystems.indexer.kicker.KickerConstants.KickerState;
import frc.robot.subsystems.indexer.kicker.KickerSubsystem;
import frc.robot.subsystems.indexer.spindexer.SpindexerConstants.SpindexerState;
import frc.robot.subsystems.indexer.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.intake.IntakeConstants.rollerState;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;

public class Shoot extends Command {
  private final FlywheelSubsystem flywheelSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final SpindexerSubsystem spindexerSubsystem;
  private final KickerSubsystem kickerSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private boolean hasSpunUp = false;

  public Shoot() {
    this.flywheelSubsystem = RobotContainer.getFlywheelSubsystem();
    this.hoodSubsystem = RobotContainer.getHoodSubsystem();
    this.spindexerSubsystem = RobotContainer.getSpindexerSubsystem();
    this.kickerSubsystem = RobotContainer.getKickerSubsystem();
    this.intakeSubsystem = RobotContainer.getIntakeSubsystem();

    addRequirements(
        flywheelSubsystem, hoodSubsystem, spindexerSubsystem, kickerSubsystem, intakeSubsystem);
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
      spindexerSubsystem.spindex(SpindexerState.IN);
      kickerSubsystem.kick(KickerState.IN);
      intakeSubsystem.roller(rollerState.INTAKE);
    }
  }

  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.stop();
    flywheelSubsystem.stop();
    spindexerSubsystem.stop();
    kickerSubsystem.stop();
    intakeSubsystem.roller(rollerState.STOP);

    RobotState.getInstance().setState(RobotStates.NEUTRAL);
  }
}
