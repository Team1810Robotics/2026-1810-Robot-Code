package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.IntakeStates;
import frc.robot.subsystems.intake.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.roller.RollerSubsystem;

public class Intake extends ParallelCommandGroup {
  private final DeploySubsystem deploySubsystem;
  private final RollerSubsystem rollerSubsystem;

  public Intake(IntakeStates state) {
    this.deploySubsystem = RobotContainer.getDeploySubsystem();
    this.rollerSubsystem = RobotContainer.getRollerSubsystem();

    RobotState.getInstance().setIntakeState(state);

    addCommands(
        deploySubsystem.deployCommand(state.getDeployState()),
        rollerSubsystem.rollerCommand(state.getRollerState()));

    addRequirements(deploySubsystem, rollerSubsystem);
  }
}
