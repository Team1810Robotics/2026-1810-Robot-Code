package frc.robot.state;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.kicker.KickerSubsystem;
import frc.robot.subsystems.indexer.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.intake.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.deploy.DeployConstants.DeployState;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.intake.roller.RollerConstants.RollerState;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.ShotParameters;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
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

  private RobotState() {
    spindexerSubsystem = RobotContainer.getSpindexerSubsystem();
 drivetrain = RobotContainer.getDrivetrain();
kickerSubsystem = RobotContainer.getKickerSubsystem();
deploySubsystem = RobotContainer.getDeploySubsystem();
rollerSubsystem = RobotContainer.getRollerSubsystem();
hoodSubsystem = RobotContainer.getHoodSubsystem();
flywheelSubsystem = RobotContainer.getFlywheelSubsystem();
  }

  public void setState(RobotStates newState) {
    this.robotState = newState;
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

  private void neutral() {
    deploySubsystem.setState(DeployState.RETRACT);
    rollerSubsystem.setState(RollerState.STOP);

    flywheelSubsystem.idle();
    hoodSubsystem.setPosition(Rotation2d.kZero);

    kickerSubsystem.stop();
    spindexerSubsystem.stop();
  }

  private void intaking() {
    deploySubsystem.setState(DeployState.DEPLOY);
    rollerSubsystem.setState(RollerState.INTAKE);

    flywheelSubsystem.idle();
    hoodSubsystem.setPosition(Rotation2d.kZero);

    kickerSubsystem.stop();
    spindexerSubsystem.stop();
  }

  private void scoringNoAgitation() {
    flywheelSubsystem.setVelocity(ShotCalculator.getInstance().calculateParameters().flywheelVelocity());
    hoodSubsystem.setPosition(ShotCalculator.getInstance().calculateParameters().hoodAngle());
  }

  private void scoringWithAgitation() {
    deploySubsystem.setState(DeployState.AGITATE);
    rollerSubsystem.setState(RollerState.INTAKE);

    flywheelSubsystem.setVelocity(ShotCalculator.getInstance().calculateParameters().flywheelVelocity());
    hoodSubsystem.setPosition(ShotCalculator.getInstance().calculateParameters().hoodAngle());
  }

  private void passing() {
    flywheelSubsystem.setVelocity(ShotCalculator.getInstance().calculateParameters().flywheelVelocity());
    hoodSubsystem.setPosition(ShotCalculator.getInstance().calculateParameters().hoodAngle());
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
