package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.deploy.DeployConstants;
import frc.robot.subsystems.intake.deploy.DeployConstants.DeployState;
import frc.robot.subsystems.intake.roller.RollerConstants;
import frc.robot.subsystems.intake.roller.RollerConstants.RollerState;

// I'm like 90% sure this doesn't do anthing but I'm scared to delete it so here we are
public enum IntakeStates {
  INTAKE(RollerConstants.RollerState.INTAKE, DeployConstants.DeployState.DEPLOY),
  OUT(RollerConstants.RollerState.OUT, DeployConstants.DeployState.DEPLOY),
  STOP(RollerConstants.RollerState.STOP, DeployConstants.DeployState.RETRACT),
  AGITATE(RollerConstants.RollerState.STOP, DeployConstants.DeployState.AGITATE);

  private RollerState rollerState;
  private DeployState deployState;

  private IntakeStates(RollerState rollerState, DeployState deployState) {
    this.rollerState = rollerState;
    this.deployState = deployState;
  }

  public DeployState getDeployState() {
    return this.deployState;
  }

  public RollerState getRollerState() {
    return this.rollerState;
  }
}
