package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeStates;
import java.util.function.BooleanSupplier;

import dev.doglog.DogLog;

public class RobotState {

  private RobotState() {}

  public RobotStates robotState = RobotStates.NEUTRAL;
  public IntakeStates intakeState = IntakeStates.STOP;

  public void setState(RobotStates state) {
    this.robotState = state;
  }

  public Command setStateCommand(RobotStates state) {
    return Commands.runOnce(() -> setState(state));
  }

  public BooleanSupplier checkRobotState(RobotStates state) {
    return () -> state == this.robotState;
  }

  public void setIntakeState(IntakeStates state) {
    this.intakeState = state;
  }

  public Command setStateCommand(IntakeStates state) {
    return Commands.runOnce(() -> setIntakeState(state));
  }

  public BooleanSupplier checkIntakeState(IntakeStates state) {
    return () -> state == this.intakeState;
  }

  public void log() {
    DogLog.log("RobotState/Robot State", robotState.toString());
    DogLog.log("RobotState/Intake State", intakeState.toString());
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
