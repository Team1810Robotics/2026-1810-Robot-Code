package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.deploy.DeployConstants;
import frc.robot.subsystems.intake.deploy.DeployConstants.deployState;
import frc.robot.subsystems.intake.roller.RollerConstants;
import frc.robot.subsystems.intake.roller.RollerConstants.RollerState;

public enum IntakeStates {
  INTAKE(RollerConstants.RollerState.INTAKE, DeployConstants.deployState.DEPLOY),
  OUT(RollerConstants.RollerState.OUT, DeployConstants.deployState.DEPLOY),
  STOP(RollerConstants.RollerState.STOP, DeployConstants.deployState.RETRACT),
  AGITATE(RollerConstants.RollerState.STOP, DeployConstants.deployState.AGITATE);

  private RollerState rollerState;
  private deployState deployState;

  private IntakeStates(RollerState rollerState, deployState deployState) {
    this.rollerState = rollerState;
    this.deployState = deployState;
  }

  public deployState getDeployState() {
    return this.deployState;
  }

  public RollerState getRollerState() {
    return this.rollerState;
  }
}
