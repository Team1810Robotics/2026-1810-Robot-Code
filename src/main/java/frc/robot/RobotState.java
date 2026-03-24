package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeStates;
import frc.robot.subsystems.intake.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.roller.RollerConstants.RollerState;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import java.util.function.BooleanSupplier;

public class RobotState {

  private RobotState() {}

  public RobotStates robotState = RobotStates.NEUTRAL;
  public IntakeStates intakeState = IntakeStates.STOP;

  public boolean killShooter = false;
  public boolean killIntake = false;

  public void setState(RobotStates state) {
    this.robotState = state;
  }

  public Command setStateCommand(RobotStates state) {
    return Commands.runOnce(() -> setState(state));
  }

  public BooleanSupplier checkRobotState(RobotStates state) {
    return () -> state == this.robotState;
  }

  public Command advanceIntakeState() {
    RollerSubsystem rollerSubsystem = RobotContainer.getRollerSubsystem();
    DeploySubsystem deploySubsystem = RobotContainer.getDeploySubsystem();

    return Commands.runOnce(
        () -> {
          IntakeStates next;

          switch (intakeState) {
            case INTAKE:
              next = IntakeStates.STOP;
              break;
            case STOP:
              next = IntakeStates.INTAKE;
              break;
            default:
              next = IntakeStates.STOP;
              break;
          }

          rollerSubsystem.roller(next.getRollerState());
          deploySubsystem.deploy(next.getDeployState());

          intakeState = next;
        });
  }

  public Command setIntakeState(IntakeStates state) {
    RollerSubsystem rollerSubsystem = RobotContainer.getRollerSubsystem();
    DeploySubsystem deploySubsystem = RobotContainer.getDeploySubsystem();

    return Commands.runOnce(
        () -> {
          rollerSubsystem.roller(
              DriverStation.isAutonomous() ? RollerState.AUTO_INTAKE : state.getRollerState());
          deploySubsystem.deploy(state.getDeployState());
        });
  }

  public BooleanSupplier checkIntakeState(IntakeStates state) {
    return () -> {
      DogLog.log("RobotState/Intake State Check", this.intakeState.toString());
      return state == this.intakeState;
    };
  }

  public void log() {
    DogLog.log("RobotState/Robot State", robotState.toString());
    DogLog.log("RobotState/Intake State", intakeState.toString());
    DogLog.log("RobotState/Kill Intake", killIntake);
    DogLog.log("RobotState/Kill Shooter", killShooter);
  }

  public static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }

    return instance;
  }

  public enum RobotStates {
    SHOOTING,
    NEUTRAL
  }
}
