package frc.robot.state;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
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

  private RobotStates robotState = RobotStates.NEUTRAL;
  private RobotStates previousState = robotState;

  // ✅ Separate state for unjam recovery
  private RobotStates preUnjamState = robotState;

  private final SpindexerSubsystem spindexerSubsystem;
  private final CommandSwerveDrivetrain drivetrain;
  private final KickerSubsystem kickerSubsystem;
  private final DeploySubsystem deploySubsystem;
  private final RollerSubsystem rollerSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;
  private final TurretSubsystem turretSubsystem;

  private boolean flywheelSpunUp = false;
  public boolean isShooterReady = false;

  private boolean indexerJammed = false;

  private boolean unjamStarted = false;
  private double unjamStartTime;
  private final double unjamTime = 0.1;

  private double lastUnjamTime = 0;
  private final double unjamCooldown = 1;

  private boolean allowStateOverride = false;

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
    double time = Timer.getFPGATimestamp();

    indexerJammed = spindexerSubsystem.isJammed();

    // ✅ Start unjam
    if (indexerJammed && !unjamStarted && time - lastUnjamTime > unjamCooldown) {
      preUnjamState = robotState; // ✅ SAVE correct state

      unjamStartTime = time;
      unjamStarted = true;
      lastUnjamTime = time;

      setState(RobotStates.REVERSE_INDEXER);
    }

    // ✅ End unjam
    if (unjamStarted && time - unjamStartTime > unjamTime) {
      unjamStarted = false;

      allowStateOverride = true;
      setState(preUnjamState); // ✅ restore correct state
      allowStateOverride = false;
    }

    // Shooter readiness logic
    if (isShootingState()) {
      ShotParameters params = ShotCalculator.getInstance().calculateParameters();

      if (flywheelSubsystem.atTargetVelocity() && !flywheelSpunUp) {
        flywheelSpunUp = true;
      }

      isShooterReady = flywheelSpunUp && turretSubsystem.atTargetAngle() && params.isValid();
    }

    DogLog.log("RobotState/Shooter Ready", isShooterReady);
    DogLog.log("RobotState/State", robotState);
    DogLog.log("RobotState/Previous State", previousState);
    DogLog.log("RobotState/Pre Unjam State", preUnjamState);
    DogLog.log("RobotState/Indexer Jammed", indexerJammed);
    DogLog.log("RobotState/Unjam Active", unjamStarted);
  }

  public void setState(RobotStates newState) {
    if (!allowStateOverride && indexerJammed && newState != RobotStates.REVERSE_INDEXER) {
      return;
    }

    if (this.robotState == newState) {
      return;
    }

    // Only update previousState for normal transitions
    if (newState != RobotStates.REVERSE_INDEXER) {
      this.previousState = this.robotState;
    }

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

  public RobotStates getPreviousState() {
    return previousState;
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
      case REVERSE_INDEXER:
        reverseIndexer();
        break;
      default:
        break;
    }
  }

  private void onStateExit() {
    if (isShootingState()) {
      flywheelSpunUp = false;
      isShooterReady = false;
    }
  }

  private void neutral() {
    deploySubsystem.setState(DeployState.RETRACT);
    rollerSubsystem.setState(RollerState.STOP);

    flywheelSubsystem.setState(FlywheelState.IDLE);
    hoodSubsystem.setState(HoodState.NEUTRAL);

    kickerSubsystem.setState(KickerState.STOP);
    spindexerSubsystem.setSpindexerState(SpindexerState.STOP);
  }

  private void intaking() {
    deploySubsystem.setState(DeployState.DEPLOY);
    rollerSubsystem.setState(RollerState.INTAKE);

    flywheelSubsystem.setState(FlywheelState.IDLE);
    hoodSubsystem.setState(HoodState.NEUTRAL);

    kickerSubsystem.setState(KickerState.STOP);
    spindexerSubsystem.setSpindexerState(SpindexerState.STOP);
  }

  private void scoringNoAgitation() {
    flywheelSubsystem.setState(FlywheelState.SCORING);
    hoodSubsystem.setState(HoodState.SCORING);

    spindexerSubsystem.setSpindexerState(SpindexerState.SHOOTING);
    kickerSubsystem.setState(KickerState.SHOOTING);
  }

  private void scoringWithAgitation() {
    deploySubsystem.setState(DeployState.AGITATE);
    rollerSubsystem.setState(RollerState.INTAKE);

    flywheelSubsystem.setState(FlywheelState.SCORING);
    hoodSubsystem.setState(HoodState.SCORING);

    spindexerSubsystem.setSpindexerState(SpindexerState.SHOOTING);
    kickerSubsystem.setState(KickerState.SHOOTING);
  }

  private void passing() {
    flywheelSubsystem.setState(FlywheelState.PASSING);
    hoodSubsystem.setState(HoodState.PASSING);

    spindexerSubsystem.setSpindexerState(SpindexerState.SHOOTING);
    kickerSubsystem.setState(KickerState.SHOOTING);
  }

  private void reverseIndexer() {
    spindexerSubsystem.setSpindexerState(SpindexerState.OUT);
    kickerSubsystem.setState(KickerState.OUT);
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
    PASSING,
    REVERSE_INDEXER
  }
}
