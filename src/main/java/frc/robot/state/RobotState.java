package frc.robot.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.kicker.KickerConstants.KickerState;
import frc.robot.subsystems.indexer.kicker.KickerSubsystem;
import frc.robot.subsystems.indexer.spindexer.SpindexerConstants.SpindexerState;
import frc.robot.subsystems.indexer.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.intake.deploy.DeployConstants.DeployState;
import frc.robot.subsystems.intake.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.roller.RollerConstants.RollerState;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.ShotParameters;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelState;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodConstants.HoodState;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;

public class RobotState {

  private RobotStates robotState;

  private final SpindexerSubsystem spindexerSubsystem;
  private final CommandSwerveDrivetrain drivetrain;
  private final KickerSubsystem kickerSubsystem;
  private final DeploySubsystem deploySubsystem;
  private final RollerSubsystem rollerSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;
  private final TurretSubsystem turretSubsystem;

  private boolean flywheelSpunUp = false;
  private boolean indexerReversed = false;
  private boolean intakeRetracted = false;

  private RobotState() {
    spindexerSubsystem = RobotContainer.getSpindexerSubsystem();
    drivetrain = RobotContainer.getDrivetrain();
    kickerSubsystem = RobotContainer.getKickerSubsystem();
    deploySubsystem = RobotContainer.getDeploySubsystem();
    rollerSubsystem = RobotContainer.getRollerSubsystem();
    hoodSubsystem = RobotContainer.getHoodSubsystem();
    flywheelSubsystem = RobotContainer.getFlywheelSubsystem();
    turretSubsystem = RobotContainer.getTurretSubsystem();
  }

  public void periodic() {
    turretSubsystem.setFieldRelativeAngle();

    if (indexerReversed) {
      kickerSubsystem.setState(KickerState.OUT);
      spindexerSubsystem.setState(SpindexerState.OUT);

      return;
    }

    if (isScoringState()) {
      ShotParameters params = ShotCalculator.getInstance().calculateScoringParameters();

      if (flywheelSubsystem.atTargetVelocity() && !flywheelSpunUp) {
        flywheelSpunUp = true;
      }

      boolean ready = flywheelSpunUp && turretSubsystem.atTargetAngle() && params.isValid();

      if (ready) {
        spindexerSubsystem.setState(SpindexerState.IN);
        kickerSubsystem.setState(KickerState.IN);

        if (robotState == RobotStates.SCORING_WITH_AGITATION) {
          deploySubsystem.setState(DeployState.AGITATE);
        }
      } else {
        spindexerSubsystem.setState(SpindexerState.STOP);
        kickerSubsystem.setState(KickerState.STOP);
      }
    }

    if (intakeRetracted && robotState == RobotStates.SCORING_NO_AGITATION) {
      deploySubsystem.setState(DeployState.RETRACT);
      rollerSubsystem.setState(RollerState.STOP);
    }
  }

  public void setState(RobotStates newState) {
    onStateExit();
    this.robotState = newState;
    applyStates();
  }

  public Command setStateCommand(RobotStates newState) {
    return Commands.runOnce(() -> setState(newState));
  }

  public RobotStates getState() {
    return robotState;
  }

  private void applyStates() {
    switch (robotState) {
      case NEUTRAL:
        neutral();
        break;
      case INTAKING:
        intaking();
        break;
      case SCORING_NO_AGITATION:
        scoringNoAgitation();
        break;
      case SCORING_WITH_AGITATION:
        scoringWithAgitation();
        break;
      case PASSING:
        passing();
        break;
      default:
        break;
    }
  }

  private void onStateExit() {
    switch (robotState) {
      case SCORING_NO_AGITATION, SCORING_WITH_AGITATION, PASSING:
        flywheelSpunUp = false;
        break;
      default:
        break;
    }
  }

  private void neutral() {
    deploySubsystem.setState(DeployState.RETRACT);
    rollerSubsystem.setState(RollerState.STOP);

    flywheelSubsystem.setState(FlywheelState.IDLE);
    hoodSubsystem.setState(HoodState.NEUTRAL);

    kickerSubsystem.setState(KickerState.STOP);
    spindexerSubsystem.setState(SpindexerState.STOP);
  }

  private void intaking() {
    deploySubsystem.setState(DeployState.DEPLOY);
    rollerSubsystem.setState(RollerState.INTAKE);

    flywheelSubsystem.setState(FlywheelState.IDLE);
    hoodSubsystem.setState(HoodState.NEUTRAL);

    kickerSubsystem.setState(KickerState.STOP);
    spindexerSubsystem.setState(SpindexerState.STOP);
  }

  private void scoringNoAgitation() {
    flywheelSubsystem.setState(FlywheelState.SCORING);
    hoodSubsystem.setState(HoodState.SCORING);
  }

  private void scoringWithAgitation() {
    deploySubsystem.setState(DeployState.AGITATE);
    rollerSubsystem.setState(RollerState.INTAKE);

    flywheelSubsystem.setState(FlywheelState.SCORING);
    hoodSubsystem.setState(HoodState.SCORING);
  }

  private void passing() {
    flywheelSubsystem.setState(FlywheelState.PASSING);
    hoodSubsystem.setState(HoodState.PASSING);
  }

  public boolean isShootingState() {
    return isPassingState() || isScoringState();
  }

  public boolean isScoringState() {
    return robotState == RobotStates.SCORING_NO_AGITATION
        || robotState == RobotStates.SCORING_WITH_AGITATION;
  }

  public boolean isPassingState() {
    return robotState == RobotStates.PASSING;
  }

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }

    return instance;
  }

  public enum RobotStates {
    NEUTRAL,
    INTAKING,
    SCORING_NO_AGITATION,
    SCORING_WITH_AGITATION,
    PASSING
  }
}
