package frc.robot.subsystems.shooter.stationary;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StaticHoodSubsystem extends SubsystemBase {
  private final TalonFX hoodMotor;
  private final DutyCycleEncoder hoodEncoder;
  private int hoodEncoderRotations = 0;
  private boolean encoderInitialized = false;
  private double lastWrappedEncoder = 0.0;
  private double lastEncoderDelta = 0.0;

  public StaticHoodSubsystem() {
    hoodMotor = new TalonFX(StaticConstants.HOOD_MOTOR_ID);
    hoodEncoder = new DutyCycleEncoder(StaticConstants.HOOD_ENCODER_DIO);
  }

  private double getWrappedHoodPosition() {
    double wrapped = 1.0 - hoodEncoder.get();

    // Keep in [0,1)
    if (wrapped >= 1.0) {
      wrapped -= 1.0;
    } else if (wrapped < 0.0) {
      wrapped += 1.0;
    }

    return wrapped;
  }

  public double getHoodRawEncoder() {
    return hoodEncoder.get();
  }

  public double getWrappedEncoder() {
    return getWrappedHoodPosition();
  }

  /**
   * Continuous hood position in "encoder turns from home", where: home ≈ 0 moving hood up increases
   * position
   */
  public double getContinuousHoodEncoder() {
    double continuous = hoodEncoderRotations + getWrappedHoodPosition();
    return continuous;
  }

  private void updateEncoderTracking() {
    double currentWrapped = getWrappedHoodPosition();

    if (!encoderInitialized) {
      lastWrappedEncoder = currentWrapped;
      encoderInitialized = true;
      lastEncoderDelta = 0.0;
      return;
    }
  }

  @Override
  public void periodic() {
    updateEncoderTracking();

    SmartDashboard.putNumber("Hood Raw Encoder", getHoodRawEncoder());
    SmartDashboard.putNumber("Hood Wrapped Encoder", getWrappedEncoder());
    SmartDashboard.putNumber("Hood Continuous Encoder", getContinuousHoodEncoder());
    SmartDashboard.putNumber("Hood Encoder Rotations", hoodEncoderRotations);
  }
}
