package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotStates;
import frc.robot.subsystems.indexer.kicker.KickerConstants.KickerState;
import frc.robot.subsystems.indexer.kicker.KickerSubsystem;
import frc.robot.subsystems.indexer.spindexer.SpindexerConstants.SpindexerState;
import frc.robot.subsystems.indexer.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.intake.roller.RollerConstants.rollerState;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.ShotParameters;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;

public class ShootNoAgitate extends Command {
  private final FlywheelSubsystem flywheelSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final SpindexerSubsystem spindexerSubsystem;
  private final KickerSubsystem kickerSubsystem;
  private final RollerSubsystem rollerSubsystem;
  private final TurretSubsystem turretSubsystem;

  private boolean isReady = false;

  public ShootNoAgitate() {
    this.flywheelSubsystem = RobotContainer.getFlywheelSubsystem();
    this.hoodSubsystem = RobotContainer.getHoodSubsystem();
    this.spindexerSubsystem = RobotContainer.getSpindexerSubsystem();
    this.kickerSubsystem = RobotContainer.getKickerSubsystem();
    this.rollerSubsystem = RobotContainer.getRollerSubsystem();
    this.turretSubsystem = RobotContainer.getTurretSubsystem();

    isReady = false;

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

    if (!isReady && flywheelSubsystem.atTargetVelocity() && turretSubsystem.atTargetAngle()) {
      isReady = true;
    }

    if (isReady) {
      spindexerSubsystem.spindex(SpindexerState.IN);
      kickerSubsystem.kick(KickerState.IN);
      rollerSubsystem.roller(rollerState.INTAKE);
    }

    DogLog.log("Shooter/HasSpunUp", isReady);

    // if (hasSpunUp && !agitateCommand.isScheduled() && Timer.getFPGATimestamp() - startTime > 7.5)
    // {
    //   CommandScheduler.getInstance().schedule(agitateCommand);
    // }
  }

  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.stop();
    flywheelSubsystem.stop();
    spindexerSubsystem.stop();
    kickerSubsystem.stop();
    rollerSubsystem.stopRoller();

    isReady = false;

    RobotState.getInstance().setState(RobotStates.NEUTRAL);
  }
}
