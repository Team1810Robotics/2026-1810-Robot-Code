package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotStates;
import frc.robot.subsystems.indexer.kicker.KickerConstants.KickerState;
import frc.robot.subsystems.indexer.kicker.KickerSubsystem;
import frc.robot.subsystems.indexer.spindexer.SpindexerConstants.SpindexerState;
import frc.robot.subsystems.indexer.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.intake.deploy.DeploySubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;

public class Shoot extends Command {
  private final FlywheelSubsystem flywheelSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final SpindexerSubsystem spindexerSubsystem;
  private final KickerSubsystem kickerSubsystem;
  private final DeploySubsystem deploySubsystem;

  private boolean hasSpunUp = false;

  private final Command agitateCommand;

  private double startTime;

  public Shoot() {
    this.flywheelSubsystem = RobotContainer.getFlywheelSubsystem();
    this.hoodSubsystem = RobotContainer.getHoodSubsystem();
    this.spindexerSubsystem = RobotContainer.getSpindexerSubsystem();
    this.kickerSubsystem = RobotContainer.getKickerSubsystem();
    this.deploySubsystem = RobotContainer.getDeploySubsystem();

    agitateCommand = deploySubsystem.agitateCommand();

    addRequirements(
        flywheelSubsystem, hoodSubsystem, spindexerSubsystem, kickerSubsystem, deploySubsystem);
  }

  @Override
  public void execute() {
    // ShotParameters params = ShotCalculator.getInstance().calculateParameters();
    // if (!params.isValid()) return;

    // hoodSubsystem.setPosition(Rotation2d.fromDegrees(20));
    flywheelSubsystem.setVelocity(RotationsPerSecond.of(50));

    if (!hasSpunUp && flywheelSubsystem.atTargetVelocity()) {
      hasSpunUp = true;
      startTime = Timer.getFPGATimestamp();
    }

    if (hasSpunUp) {
      spindexerSubsystem.spindex(SpindexerState.IN);
      kickerSubsystem.kick(KickerState.IN);
    }

    if (hasSpunUp && !agitateCommand.isScheduled() && Timer.getFPGATimestamp() - startTime > 2) {
      CommandScheduler.getInstance().schedule(agitateCommand);
    }
  }

  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.stop();
    flywheelSubsystem.stop();
    spindexerSubsystem.stop();
    kickerSubsystem.stop();

    RobotState.getInstance().setState(RobotStates.NEUTRAL);
  }
}
