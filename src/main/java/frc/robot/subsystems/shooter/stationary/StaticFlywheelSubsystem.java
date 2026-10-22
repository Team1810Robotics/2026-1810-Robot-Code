package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class StaticFlywheelSubsystem extends SubsystemBase {
  private static final double[] EMPTY_FRAME = new double[0];
  private static final String CSV_HEADER =
      "point,axis,x_deg,tx_deg,ty_deg,tag_id,setpoint_rps,leader_rps,follower_abs_rps";

  private final CANBus bus =
      CAN_BUS_NAME.equals("rio") ? CANBus.roboRIO() : new CANBus(CAN_BUS_NAME);
  private final TalonFX leader = new TalonFX(LEADER_MOTOR_ID, bus);
  private final TalonFX follower = new TalonFX(FOLLOWER_MOTOR_ID, bus);
  private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
  private final StatusSignal<AngularVelocity> followerVelocity = follower.getVelocity();
  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0.0).withSlot(0).withEnableFOC(false);
  private final Follower followerRequest = new Follower(LEADER_MOTOR_ID, FOLLOWER_ALIGNMENT);

  private final NetworkTable limelight =
      NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);
  // t2d gives tx, ty, target-valid and tag ID from one camera frame.
  private final NetworkTableEntry frameEntry = limelight.getEntry("t2d");
  private final NetworkTableEntry heartbeatEntry = limelight.getEntry("hb");
  private final FlywheelModel model =
      new FlywheelModel(
          QUADRATIC_A, QUADRATIC_B, QUADRATIC_C,
          CALIBRATED_MIN_X_DEGREES, CALIBRATED_MAX_X_DEGREES,
          MAX_FLYWHEEL_RPS, QUADRATIC_READY);

  private final List<String> recordedRows = new ArrayList<>();
  // getLog() starts DataLogManager if necessary. The log writer handles disk flushing.
  private final StringLogEntry pointLog =
      new StringLogEntry(DataLogManager.getLog(), "/Flywheel/Calibration/CSV");

  private boolean configured;
  private boolean motorSignalsFresh;
  private boolean validTarget;
  private double leaderRps;
  private double followerRps;
  private double requestedRps;
  private double manualRps;
  private double testX;
  private double tx = Double.NaN;
  private double ty = Double.NaN;
  private int tagId = -1;
  private double previousHeartbeat = Double.NaN;
  private double lastHeartbeatTime = Double.NEGATIVE_INFINITY;
  private long previousFrameChange;
  private double lastFrameTime = Double.NEGATIVE_INFINITY;
  private double atSpeedSince = Double.NaN;
  private double lastDashboardTime = Double.NEGATIVE_INFINITY;
  private String state = "Stopped";
  private String targetStatus = "Waiting for camera";
  private String recordStatus = "Press A after a successful shot and speed recovery";

  public StaticFlywheelSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = LEADER_INVERSION;
    config.Feedback.SensorToMechanismRatio = MOTOR_ROTATIONS_PER_FLYWHEEL_ROTATION;
    config.Slot0.kS = KS;
    config.Slot0.kV = KV;
    config.Slot0.kP = KP;
    config.Slot0.kI = KI;
    config.Slot0.kD = KD;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    boolean leaderConfigured = leader.getConfigurator().apply(config).isOK();
    boolean followerConfigured = follower.getConfigurator().apply(config).isOK();
    boolean leaderSignalConfigured = leaderVelocity.setUpdateFrequency(SIGNAL_FREQUENCY_HZ).isOK();
    boolean followerSignalConfigured = followerVelocity.setUpdateFrequency(SIGNAL_FREQUENCY_HZ).isOK();
    // Phoenix 6 2026 followers copy MotorVoltage for voltage control requests.
    boolean voltageSignalConfigured =
        leader.getMotorVoltage().setUpdateFrequency(SIGNAL_FREQUENCY_HZ).isOK();
    configured = leaderConfigured && followerConfigured && leaderSignalConfigured
        && followerSignalConfigured && voltageSignalConfigured;
    if (!configured) {
      DriverStation.reportError("Flywheel configuration failed. Check CAN IDs/bus/power.", false);
    }

    pointLog.append(CSV_HEADER);
    SmartDashboard.putString("Flywheel/Points CSV", CSV_HEADER);
    SmartDashboard.putString("Flywheel/Last Point", "No points recorded");
    stop();
  }

  public void adjustManualRps(double delta) {
    if (Double.isFinite(delta)) {
      manualRps = MathUtil.clamp(manualRps + delta, 0.0, MAX_FLYWHEEL_RPS);
    }
  }

  public void resetManualRps() {
    manualRps = 0.0;
  }

  /** Changes only a dashboard equation preview; never writes fake data to the camera. */
  public void adjustTestX(double deltaDegrees) {
    if (Double.isFinite(deltaDegrees)) {
      testX = MathUtil.clamp(testX + deltaDegrees, -90.0, 90.0);
        }
    }
  private boolean speedsWithinTolerance() {
    return configured && motorSignalsFresh && requestedRps > 0.0
        && Math.abs(leaderRps - requestedRps) <= SPEED_TOLERANCE_RPS
        && Math.abs(Math.abs(followerRps) - requestedRps) <= SPEED_TOLERANCE_RPS;
  }


  /** Speed check only; the indexer must also check hood position and aim. */
  public boolean isAtSpeed() {
    return DriverStation.isEnabled() && speedsWithinTolerance()
        && Double.isFinite(atSpeedSince)
        && Timer.getFPGATimestamp() - atSpeedSince >= AT_SPEED_TIME_SECONDS;
  }
  
  /** Call only after a successful manual test shot. Records real camera data. */
  public void recordTestPoint() {
    if (!validTarget) {
      recordStatus = "Not recorded: " + targetStatus;
    } else if (!isAtSpeed()) {
      recordStatus = "Not recorded: hold trigger and wait for both wheels to settle";
    } else if (recordedRows.size() >= MAX_RECORDED_POINTS) {
      recordStatus = "Point table full; copy/export the current points";
    } else {
      String row = String.format(Locale.ROOT, "%d,%s,%.6f,%.6f,%.6f,%d,%.6f,%.6f,%.6f",
          recordedRows.size() + 1, VISION_AXIS, getLiveX(), tx, ty, tagId,
          requestedRps, leaderRps, Math.abs(followerRps));
      recordedRows.add(row);
      pointLog.append(row);
      SmartDashboard.putString("Flywheel/Last Point", row);
      SmartDashboard.putString("Flywheel/Points CSV", CSV_HEADER + "\n" + String.join("\n", recordedRows));
      recordStatus = "Recorded point " + recordedRows.size();
    }
  }

  /** Schedule this explicitly from autonomous; it stops when canceled/interrupted. */
  public Command automaticCommand() {
    return runEnd(this::runFromVision, this::stop).withName("VisionFlywheel");
  }

  private boolean tagIsAllowed(int id) {
    for (int allowed : ALLOWED_TAG_IDS) {
      if (allowed == id) {
        return true;
      }
    }
    return false;
  }

  private void updateVision(double now) {
    double heartbeat = heartbeatEntry.getDouble(Double.NaN);
    // Require an observed heartbeat CHANGE, not just an old cached value.
    if (Double.isFinite(heartbeat) && Double.isFinite(previousHeartbeat)
        && heartbeat != previousHeartbeat) {
      lastHeartbeatTime = now;
    }
    previousHeartbeat = heartbeat;
    long frameChange = frameEntry.getLastChange();
    if (frameChange != 0 && frameChange != previousFrameChange) {
      lastFrameTime = now;
    }
    previousFrameChange = frameChange;
    double[] frame = frameEntry.getDoubleArray(EMPTY_FRAME);
    boolean completeFrame = frame.length >= 10;
    tx = completeFrame ? frame[4] : Double.NaN;
    ty = completeFrame ? frame[5] : Double.NaN;
    tagId = completeFrame && Double.isFinite(frame[9]) ? (int) frame[9] : -1;
    boolean fresh = now - lastHeartbeatTime <= VISION_TIMEOUT_SECONDS
        && now - lastFrameTime <= VISION_TIMEOUT_SECONDS;
    boolean detected = completeFrame && frame[0] == 1.0
        && Double.isFinite(tx) && Double.isFinite(ty) && tagId > 0;
    validTarget = fresh && detected && tagIsAllowed(tagId);
    targetStatus = !fresh ? "Camera/result stale or missing t2d/hb"
        : !detected ? "No valid AprilTag"
        : !tagIsAllowed(tagId) ? "Tag not allowed: configure ALLOWED_TAG_IDS"
        : "Valid target";
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    // Nonblocking refresh of cached CAN signals; never waitForUpdate in this loop.
    boolean refreshOk = BaseStatusSignal.refreshAll(leaderVelocity, followerVelocity).isOK();
    leaderRps = leaderVelocity.getValueAsDouble();
    followerRps = followerVelocity.getValueAsDouble();
    motorSignalsFresh = refreshOk
        && leaderVelocity.getTimestamp().getLatency() <= MOTOR_SIGNAL_TIMEOUT_SECONDS
        && followerVelocity.getTimestamp().getLatency() <= MOTOR_SIGNAL_TIMEOUT_SECONDS;
    updateVision(now);
    if (DriverStation.isDisabled()) {
      stop();
    }
    if (speedsWithinTolerance()) {
      if (!Double.isFinite(atSpeedSince)) {
        atSpeedSince = now;
      }
    } else {
      atSpeedSince = Double.NaN;
    }
    if (now - lastDashboardTime >= DASHBOARD_PERIOD_SECONDS) {
      lastDashboardTime = now;
      SmartDashboard.putBoolean("Flywheel/CAN Config OK", configured);
      SmartDashboard.putBoolean("Flywheel/Motor Signals Fresh", motorSignalsFresh);
      SmartDashboard.putBoolean("Flywheel/Valid Target", validTarget);
      SmartDashboard.putBoolean("Flywheel/At Speed", isAtSpeed());
      SmartDashboard.putBoolean("Flywheel/Quadratic Enabled", QUADRATIC_READY);
      SmartDashboard.putString("Flywheel/State", state);
      SmartDashboard.putString("Flywheel/Target Status", targetStatus);
      SmartDashboard.putString("Flywheel/Input Axis", VISION_AXIS.name());
      SmartDashboard.putString("Flywheel/Record Status", recordStatus);
      SmartDashboard.putNumber("Flywheel/TX deg", tx);
      SmartDashboard.putNumber("Flywheel/TY deg", ty);
      SmartDashboard.putNumber("Flywheel/Tag ID", tagId);
      SmartDashboard.putNumber("Flywheel/Live X deg", getLiveX());
      SmartDashboard.putNumber("Flywheel/Manual RPS", manualRps);
      SmartDashboard.putNumber("Flywheel/Requested RPS", requestedRps);
      SmartDashboard.putNumber("Flywheel/Leader RPS", leaderRps);
      SmartDashboard.putNumber("Flywheel/Follower Abs RPS", Math.abs(followerRps));
      SmartDashboard.putNumber("Flywheel/Leader RPM", leaderRps * 60.0);
      SmartDashboard.putNumber("Flywheel/Point Count", recordedRows.size());
      SmartDashboard.putNumber("Flywheel/Test X deg", testX);
      SmartDashboard.putNumber("Flywheel/Test Equation RPS", model.rpsFor(testX));
    }
  }
}
}
}