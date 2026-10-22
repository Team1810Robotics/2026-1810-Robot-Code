
package frc.robot.subsystems.shooter.stationary;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StaticHoodSubsystem extends SubsystemBase {

  private final TalonFX hoodMotor =
      new TalonFX(HoodConstants.HOOD_MOTOR_ID);

  private final DutyCycleEncoder hoodEncoder =
      new DutyCycleEncoder(HoodConstants.HOOD_ENCODER_DIO);

  private final PIDController hoodPID =
      new PIDController(
          HoodConstants.kP,
          HoodConstants.kI,
          HoodConstants.kD);

  private double targetPosition = 0.0;
  private boolean holdingPosition = false;

  private int telemetryCounter = 0;

  public StaticHoodSubsystem() {
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);

    hoodPID.setTolerance(
        HoodConstants.POSITION_TOLERANCE);

    // Only initialize the setpoint once the position
    // is calibrated and the encoder is connected.
    if (isReady()) {
      holdCurrentPosition();
    }
  }

  // Absolute encoder reading: approximately [0, 1].
  public double getRawPosition() {
    return hoodEncoder.get();
  }

  public boolean isReady() {
    return HoodConstants.isCalibrated()
        && HoodConstants.MOTOR_DIRECTION_VERIFIED
        && hoodEncoder.isConnected();
  }

  // Convert raw absolute encoder values into a
  // positive position measured from the lower limit.
  public double getPosition() {

    if (!HoodConstants.isCalibrated()
        || !hoodEncoder.isConnected()) {
      return Double.NaN;
    }

    double delta =
        getRawPosition() - HoodConstants.ZERO_RAW;

    if (!HoodConstants.ENCODER_INCREASES_UP) {
      delta = -delta;
    }

    // Normalize wraparound to [0, 1).
    return ((delta % 1.0) + 1.0) % 1.0;
  }

  // Hold the hood wherever it currently is.
  public void holdCurrentPosition() {
    if (!isReady()) {
      holdingPosition = false;
      hoodMotor.set(0.0);
      return;
    }

    double position = getPosition();

    if (!Double.isFinite(position)
        || position < HoodConstants.MIN_POSITION
        || position > HoodConstants.MAX_POSITION) {
      holdingPosition = false;
      hoodMotor.set(0.0);
      return;
    }

    targetPosition = position;
    hoodPID.reset();
    holdingPosition = true;
  }

  // Move to a requested encoder position.
  public void setPoint(double position) {

    if (!isReady() || !Double.isFinite(position)) {
      return;
    }

    targetPosition = MathUtil.clamp(
        position,
        HoodConstants.MIN_POSITION,
        HoodConstants.MAX_POSITION);

    holdingPosition = true;
  }

  public double getSetPoint() {
    return targetPosition;
  }

  public boolean isAtSetPoint() {
    return isReady()
        && holdingPosition
        && Double.isFinite(getPosition())
        && Math.abs(targetPosition - getPosition())
            <= HoodConstants.POSITION_TOLERANCE;
  }

  public void stopHood() {
    holdingPosition = false;
    hoodPID.reset();
    hoodMotor.set(0.0);
  }

  @Override
  public void periodic() {

    if (!DriverStation.isEnabled() || !isReady()) {
      hoodMotor.set(0.0);

    } else if (!holdingPosition) {
      hoodMotor.set(0.0);

    } else {

      double currentPosition = getPosition();

      // Reject invalid or out-of-range encoder readings.
      if (!Double.isFinite(currentPosition)
          || currentPosition < HoodConstants.MIN_POSITION
          || currentPosition > HoodConstants.MAX_POSITION) {

        stopHood();

      } else {

        double pidOutput =
            hoodPID.calculate(
                currentPosition, targetPosition);

        pidOutput = MathUtil.clamp(
            pidOutput,
            -HoodConstants.MAX_OUTPUT,
            HoodConstants.MAX_OUTPUT);

        hoodMotor.set(
            pidOutput * HoodConstants.MOTOR_DIRECTION);
      }
    }

    // Dashboard updates at approximately 5 Hz,
    // rather than every 20 ms scheduler cycle.
    telemetryCounter++;

    if (telemetryCounter >= 10) {
      telemetryCounter = 0;

      SmartDashboard.putNumber(
          "Hood/Raw Encoder", getRawPosition());

      SmartDashboard.putNumber(
          "Hood/Position", getPosition());

      SmartDashboard.putNumber(
          "Hood/Setpoint", targetPosition);

      SmartDashboard.putBoolean(
          "Hood/Encoder Connected",
          hoodEncoder.isConnected());

      SmartDashboard.putBoolean(
          "Hood/Calibrated",
          HoodConstants.isCalibrated());

      SmartDashboard.putBoolean(
          "Hood/Ready", isReady());

      SmartDashboard.putBoolean(
          "Hood/At Setpoint", isAtSetPoint());
    }
  }
}
