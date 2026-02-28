package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
  private final TalonFX hoodMotor;
  private final CANcoder hoodEncoder;

  private final DoubleSubscriber tuningTarget = DogLog.tunable("Hood/TuningTarget", 0.0);

  private boolean isTuning = false;

  private Command tuningCommand = setPositionCommand(Rotation2d.fromDegrees(tuningTarget.get()));

  public HoodSubsystem() {
    hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID);
    hoodEncoder = new CANcoder(HoodConstants.HOOD_ENCODER_ID);

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 120;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 40;

    cfg.Feedback.SensorToMechanismRatio = HoodConstants.GEAR_RATIO_MOTOR;

    cfg.Slot0.kP = HoodConstants.kP;

    cfg.Slot0.kS = HoodConstants.kS;

    hoodMotor.getConfigurator().apply(cfg);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = -.262;

    hoodEncoder.getConfigurator().apply(encoderConfig);

    lastEncoderRaw = 0;
    unwrappedEncoder = 0;

    hoodMotor.setPosition(Rotations.of(0));
  }

  public void setPosition(Rotation2d position) {
    hoodMotor.setControl(new PositionVoltage(position.getRotations()));
  }

  public Command setPositionCommand(Rotation2d position) {
    return Commands.startEnd(() -> setPosition(position), () -> stop(), this);
  }

  public Rotation2d getMotorPosition() {
    return Rotation2d.fromRotations(hoodMotor.getPosition().getValueAsDouble());
  }

  private double lastEncoderRaw = 0.0;
  private double unwrappedEncoder = 0.0;

  public Rotation2d getEncoderPosition() {
    double raw = hoodEncoder.getPosition().getValueAsDouble();

    double delta = raw - lastEncoderRaw;

    if (delta > 0.5) { // jumped from 1 -> 0
      delta -= 1.0;
    } else if (delta < -0.5) { // jumped from 0 -> 1
      delta += 1.0;
    }

    unwrappedEncoder += delta;
    lastEncoderRaw = raw;

    double rotations = unwrappedEncoder / HoodConstants.GEAR_RATIO_ENCODER;

    return Rotation2d.fromRotations(rotations);
  }

  public void seedMotorFromAbsolute() {
    hoodMotor.setPosition(getEncoderPosition().getRotations());
  }

  public void log() {
    DogLog.log("Hood/MotorPosition", getMotorPosition().getDegrees(), Degrees);
    DogLog.log("Hood/EncoderPosition", getEncoderPosition().getDegrees(), Degrees);
    DogLog.log(
        "Hood/Raw Encoder Position", hoodEncoder.getPosition().getValueAsDouble(), Rotations);
    DogLog.log("Hood/Volts", hoodMotor.getMotorVoltage().getValueAsDouble(), Volts);
  }

  public void stop() {
    hoodMotor.stopMotor();
  }

  public Command zero() {
    return Commands.sequence(
        Commands.run(() -> hoodMotor.setVoltage(-1), this).withTimeout(2),
        Commands.runOnce(() -> hoodMotor.setPosition(Degrees.of(0)), this));
  }

  @Override
  public void periodic() {
    log();
    tuningCommand = setPositionCommand(Rotation2d.fromDegrees(tuningTarget.get()));

    if (tuningTarget.get() != 0) {
      isTuning = true;
      CommandScheduler.getInstance().schedule(tuningCommand);
    }

    if (isTuning && tuningTarget.get() == 0) {
      isTuning = false;
      CommandScheduler.getInstance().cancel(tuningCommand);
    }
  }
}
