package frc.robot.commands;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;

/** Default teleop command. All adjustments remain available while the trigger is held. */
public class StaticFlywheelCommand extends Command {
  private final FlywheelSubsystem flywheel;
  private final XboxController operator;
  private boolean automatic;
  private boolean previousA;
  private boolean previousB;
  private boolean previousX;
  private boolean previousY;
  private int previousPov = -1;

  public StaticFlywheelCommand(FlywheelSubsystem flywheel, XboxController operator) {
    this.flywheel = flywheel;
    this.operator = operator;
    addRequirements(flywheel);
  }

  @Override
  public void initialize() {
    automatic = false;
    samplePreviousButtons();
    SmartDashboard.putString("Flywheel/Selected Mode", "MANUAL");
    flywheel.stop();
  }

  private void samplePreviousButtons() {
    previousA = operator.getAButton();
    previousB = operator.getBButton();
    previousX = operator.getXButton();
    previousY = operator.getYButton();
    previousPov = operator.getPOV();
  }

  @Override
  public void execute() {
    // An explicitly scheduled automaticCommand() handles autonomous.
    if (!DriverStation.isTeleopEnabled() || !operator.isConnected()) {
      flywheel.stop();
      samplePreviousButtons();
      return;
    }

    boolean a = operator.getAButton();
    boolean b = operator.getBButton();
    boolean x = operator.getXButton();
    boolean y = operator.getYButton();
    int pov = operator.getPOV();

    if (x && !previousX) {
      automatic = !automatic;
      SmartDashboard.putString("Flywheel/Selected Mode", automatic ? "VISION" : "MANUAL");
    }
    // B takes priority over X and the D-pad for this cycle.
    if (b && !previousB) {
      automatic = false;
      flywheel.resetManualRps();
      SmartDashboard.putString("Flywheel/Selected Mode", "MANUAL");
    } else if (pov != previousPov) {
      double step = operator.getLeftBumperButton() ? FINE_MANUAL_STEP_RPS : MANUAL_STEP_RPS;
      if (pov == 0 && !automatic) {
        flywheel.adjustManualRps(step);
      } else if (pov == 180 && !automatic) {
        flywheel.adjustManualRps(-step);
      } else if (pov == 90) {
        flywheel.adjustTestX(TEST_X_STEP_DEGREES);
      } else if (pov == 270) {
        flywheel.adjustTestX(-TEST_X_STEP_DEGREES);
      }
    }

    if (y && !previousY) {
      flywheel.copyLiveXToTest();
    }

    boolean runRequested = operator.getRightTriggerAxis() > RUN_TRIGGER_THRESHOLD;
    if (!runRequested) {
      flywheel.stop();
    } else if (automatic) {
      flywheel.runFromVision();
    } else {
      flywheel.runManual();
    }

    if (a && !previousA && !automatic) {
      flywheel.recordTestPoint();
    }

    previousA = a;
    previousB = b;
    previousX = x;
    previousY = y;
    previousPov = pov;
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
  }
}