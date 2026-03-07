package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.deploy.DeployConstants;
import frc.robot.subsystems.intake.deploy.DeployConstants.deployState;
import frc.robot.subsystems.intake.roller.RollerConstants;
import frc.robot.subsystems.intake.roller.RollerConstants.rollerState;

public enum IntakeStates {
  INTAKE(RollerConstants.rollerState.INTAKE, DeployConstants.deployState.DEPLOY),
  OUT(RollerConstants.rollerState.OUT, DeployConstants.deployState.DEPLOY),
  STOP(RollerConstants.rollerState.STOP, DeployConstants.deployState.RETRACT),
  AGITATE(RollerConstants.rollerState.STOP, DeployConstants.deployState.AGITATE);

  private rollerState rollerState;
  private deployState deployState;

  private IntakeStates(rollerState rollerState, deployState deployState) {
    this.rollerState = rollerState;
    this.deployState = deployState;
  }

  public deployState getDeployState() {
    return this.deployState;
  }

  public rollerState getRollerState() {
    return this.rollerState;
  }
}
