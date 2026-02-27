package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;

public class RobotState {

  private RobotState() {}

  public RobotStates robotState = RobotStates.NEUTRAL;

  public BooleanSupplier stateIsNeutral = () -> robotState == RobotStates.NEUTRAL;
  public BooleanSupplier stateIsShooting = () -> robotState == RobotStates.SHOOTING;

  public Command setStateCommand(RobotStates state) {
    return Commands.runOnce(() -> setState(state));
  }

  public void setState(RobotStates state) {
    this.robotState = state;
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
