package frc.robot.commands;

import dev.doglog.DogLog;
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
import frc.robot.subsystems.intake.roller.RollerConstants.RollerState;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.ShotParameters;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;

public class ShootWithAgitate extends Command {
  private final FlywheelSubsystem flywheelSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final SpindexerSubsystem spindexerSubsystem;
  private final KickerSubsystem kickerSubsystem;
  private final RollerSubsystem rollerSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final DeploySubsystem deploySubsystem;

  private boolean isReady = false;

  private final Command agitateCommand;

  public ShootWithAgitate() {
    this.flywheelSubsystem = RobotContainer.getFlywheelSubsystem();
    this.hoodSubsystem = RobotContainer.getHoodSubsystem();
    this.spindexerSubsystem = RobotContainer.getSpindexerSubsystem();
    this.kickerSubsystem = RobotContainer.getKickerSubsystem();
    this.rollerSubsystem = RobotContainer.getRollerSubsystem();
    this.turretSubsystem = RobotContainer.getTurretSubsystem();
    this.deploySubsystem = RobotContainer.getDeploySubsystem();

    isReady = false;

    agitateCommand = deploySubsystem.agitateCommand();

    addRequirements(
        flywheelSubsystem, hoodSubsystem, spindexerSubsystem, kickerSubsystem, rollerSubsystem);
  }

  @Override
  public void initialize() {
    isReady = false;
  }

  @Override
  public void execute() {
    ShotParameters params = ShotCalculator.getInstance().calculateParameters();
    if (!params.isValid()) return;

    hoodSubsystem.setPosition(params.hoodAngle());
    flywheelSubsystem.setVelocity(params.flywheelVelocity());

    if (!isReady && flywheelSubsystem.atTargetVelocity()) {
      isReady = true;
    }

    if (!turretSubsystem.atTargetAngle()) return;

    if (isReady) {
      spindexerSubsystem.spindex(SpindexerState.IN);
      kickerSubsystem.kick(KickerState.IN);
      rollerSubsystem.roller(RollerState.INTAKE);
    }

    DogLog.log("Shooter/HasSpunUp", isReady);

    if (isReady && !agitateCommand.isScheduled()) {
      CommandScheduler.getInstance().schedule(agitateCommand);
    }
  }

  @Override
  public boolean isFinished() {
    if (RobotState.getInstance().killShooter) return true;

    return false;
  }

  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.stop();
    flywheelSubsystem.stop();
    spindexerSubsystem.stop();
    kickerSubsystem.stop();
    rollerSubsystem.stop();

    isReady = false;

    RobotState.getInstance().setState(RobotStates.NEUTRAL);
  }
}
