package frc.robot.subsystems.shooter.stationary;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public final class StaticConstants {

  // Hardware - replace with actual wiring values.
  public static final int HOOD_MOTOR_ID = 16;
  public static final int HOOD_ENCODER_DIO = 35;

  public static final String LIMELIGHT_NAME = "limelight";

  // Replace with the hub AprilTag ID you intend to track.
  // -1 disables automatic aiming until configured.
  public static final int TARGET_TAG_ID = -1;

  // Calibration:
  // Raw absolute encoder value at the LOWEST hood position.
  // Set after measuring on your actual robot.
  public static final double ZERO_RAW = Double.NaN;

  // True if raw encoder values increase as the hood raises.
  public static final boolean ENCODER_INCREASES_UP = true;

  // Measured hood travel in encoder rotations, from
  // lowest position to highest position.
  public static final double MAX_POSITION = Double.NaN;

  public static final double MIN_POSITION = 0.0;

  // PID gains - starting values, not finalized tuning.
  public static final double kP = 0.5;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  // Conservative initial motor output limit.
  public static final double MAX_OUTPUT = 0.10;

  public static final double POSITION_TOLERANCE = 0.005;

  // Change to -1 if a positive TalonFX output lowers
  // the hood instead of raising it.
  public static final double MOTOR_DIRECTION = 1.0;

  // Set true only after verifying the motor and encoder
  // move in the expected directions.
  public static final boolean MOTOR_DIRECTION_VERIFIED = false;

  // The complete hood range must be less than one
  // encoder revolution and uniquely identifiable.
  public static boolean isCalibrated() {
    return Double.isFinite(ZERO_RAW)
        && ZERO_RAW >= 0.0
        && ZERO_RAW <= 1.0
        && Double.isFinite(MAX_POSITION)
        && MAX_POSITION > 0.0
        && MAX_POSITION < 1.0;
  }

  // Add measured pairs: { Limelight ty, hood position }.
  // Sort in ascending order of ty.
  public static final double[][] SHOT_TABLE = {
    // {-10.0, 0.05},
    // {  0.0, 0.15},
    // { 10.0, 0.25}
  };

  // Leave false until every row uses actual shot data.
  public static final boolean SHOT_TABLE_CALIBRATED = false;
  ////////////////////////////////////////////////////////////////////


public final class FlywheelConstants {
  private FlywheelConstants() {}

  // EXAMPLE IDs: replace these with the IDs on your robot.
  public static final int LEADER_MOTOR_ID = 18;
  public static final int FOLLOWER_MOTOR_ID = 14;
  public static final String CAN_BUS_NAME = "rio";

  public static final InvertedValue LEADER_INVERSION =
      InvertedValue.CounterClockwise_Positive;
  // Opposed is for mirrored motors. Verify the actual wheel directions at low speed.
  public static final MotorAlignmentValue FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed;

  // Motor rotations / flywheel rotation. 1.0 means a direct drive.
  // Both motors must use the same gearing for this follower implementation.
  public static final double MOTOR_ROTATIONS_PER_FLYWHEEL_ROTATION = 1.0;

  // STARTING VALUES, not measured gains for your mechanism. Tune on the robot.
  public static final double KS = 0.0;
  public static final double KV = 0.12; // Volts per flywheel RPS; depends on gearing.
  public static final double KP = 0.10;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
  public static final double STATOR_CURRENT_LIMIT_AMPS = 80.0;

  // EXAMPLE ceiling: set this to a verified limit for your wheels and gearing.
  public static final double MAX_FLYWHEEL_RPS = 60.0;
  public static final double MANUAL_STEP_RPS = 1.0;
  public static final double FINE_MANUAL_STEP_RPS = 0.25;
  public static final double TEST_X_STEP_DEGREES = 0.5;
  public static final double SPEED_TOLERANCE_RPS = 1.0;
  public static final double AT_SPEED_TIME_SECONDS = 0.20;
  public static final double MOTOR_SIGNAL_TIMEOUT_SECONDS = 0.10;
  public static final double SIGNAL_FREQUENCY_HZ = 50.0;

  public static final String LIMELIGHT_NAME = "limelight";
  public static final double VISION_TIMEOUT_SECONDS = 0.25;

  public enum VisionAxis {
    TX,
    TY
  }

  // TX matches the requested equation. TY is recommended for range-based shots.
  // Changing the axis requires collecting/refitting the calibration data.
  public static final VisionAxis VISION_AXIS = VisionAxis.TX;

  // Fill in the desired hub tag ID(s). Empty means NO tags are accepted.
  // Start calibration with ONE tag; other heights/offsets may need separate fits.
  public static final int[] ALLOWED_TAG_IDS = {};

  // RPS = A*x*x + B*x + C. Replace these after collecting successful shots.
  public static final double QUADRATIC_A = 0.0;
  public static final double QUADRATIC_B = 0.0;
  public static final double QUADRATIC_C = 0.0;
  public static final double CALIBRATED_MIN_X_DEGREES = 0.0;
  public static final double CALIBRATED_MAX_X_DEGREES = 0.0;
  public static final boolean QUADRATIC_READY = false;

  public static final int MAX_RECORDED_POINTS = 100;
  public static final double DASHBOARD_PERIOD_SECONDS = 0.10;
  public static final int OPERATOR_CONTROLLER_PORT = 1;
  public static final double RUN_TRIGGER_THRESHOLD = 0.20;
}

}
