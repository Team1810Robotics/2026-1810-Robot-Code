
package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.stationary.HoodConstants;
import frc.robot.subsystems.shooter.stationary.StaticHoodSubsystem;

public class StaticHoodCommand extends Command {

  private final StaticHoodSubsystem hood;
  private final NetworkTable limelight;

  public StaticHoodCommand(StaticHoodSubsystem hood) {
    this.hood = hood;

    limelight = NetworkTableInstance.getDefault()
        .getTable(HoodConstants.LIMELIGHT_NAME);

    addRequirements(hood);
  }

  @Override
  public void initialize() {
    // Start by holding the current physical position.
    hood.holdCurrentPosition();
  }

  @Override
  public void execute() {

    if (!hood.isReady()) {
      return;
    }

    if (!HoodConstants.SHOT_TABLE_CALIBRATED
        || HoodConstants.SHOT_TABLE.length < 2
        || HoodConstants.TARGET_TAG_ID < 0) {
      return;
    }

    double tv = limelight
        .getEntry("tv").getDouble(0.0);

    int tagID = (int) limelight
        .getEntry("tid").getDouble(-1.0);

    // If the Limelight has no valid selected
    // target, keep the previous hood setpoint.
    if (tv < 1.0 || tagID != HoodConstants.TARGET_TAG_ID) {
      return;
    }

    double ty = limelight
        .getEntry("ty").getDouble(Double.NaN);

    if (!Double.isFinite(ty)) {
      return;
    }

    double desiredPosition =
        calculateHoodPosition(ty);

    if (Double.isFinite(desiredPosition)) {
      hood.setPoint(desiredPosition);
    }
  }

  private double calculateHoodPosition(double ty) {

    double[][] table = HoodConstants.SHOT_TABLE;

    if (table.length < 2) {
      return Double.NaN;
    }

    // Outside the measured range: do not extrapolate.
    // Use the nearest calibrated shot position.
    if (ty <= table[0][0]) {
      return table[0][1];
    }

    if (ty >= table[table.length - 1][0]) {
      return table[table.length - 1][1];
    }

    // Find two surrounding samples.
    for (int i = 0; i < table.length - 1; i++) {

      double ty1 = table[i][0];
      double ty2 = table[i + 1][0];

      double pos1 = table[i][1];
      double pos2 = table[i + 1][1];

      if (ty >= ty1 && ty <= ty2 && ty2 > ty1) {

        double ratio =
            (ty - ty1) / (ty2 - ty1);

        return pos1 + ratio * (pos2 - pos1);
      }
    }

    return Double.NaN;
  }

  @Override
  public void end(boolean interrupted) {
    hood.stopHood();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
