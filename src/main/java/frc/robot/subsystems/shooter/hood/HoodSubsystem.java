package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.ShotParameters;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class HoodSubsystem extends SubsystemBase {
  private final TalonFX hoodMotor;
  private final CANcoder hoodEncoder;

  private final DoubleSubscriber tuningTarget = DogLog.tunable("Hood/TuningTarget", 0.0);

  private boolean isTuning = false;

  public final DoubleSubscriber kP = DogLog.tunable("Hood/Gains/kP", HoodConstants.kP);
  public final DoubleSubscriber kD = DogLog.tunable("Hood/Gains/kD", 0.0);

  public final DoubleSubscriber kS = DogLog.tunable("Hood/Gains/kS", HoodConstants.kS);

  private double lastkP, lastkD, lastkS;

  private Command tuningCommand;

  private Rotation2d position;

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
    if (position != null) {
      return position;
    }

    position = Rotation2d.fromRotations(hoodMotor.getPosition().getValueAsDouble());
    return position;
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

  public Command applyVoltageCommand(double volts) {
    return Commands.run(() -> hoodMotor.setVoltage(volts), this).finallyDo(() -> stop());
  }

  public void log() {
    DogLog.log("Hood/Position", getMotorPosition().getDegrees(), Degrees);
    DogLog.log("Hood/Volts", hoodMotor.getMotorVoltage().getValueAsDouble(), Volts);
  }

  public void stop() {
    hoodMotor.stopMotor();
  }

  private double startTime = 0;

  // thank you scream
  public Command zero() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
        applyVoltageCommand(-1.0)
            .withDeadline(
                new WaitUntilCommand(() -> ((Timer.getFPGATimestamp() - startTime) > 1.0))),
        new InstantCommand(() -> hoodMotor.setPosition(Rotations.of(0))));
  }

  public void updateGains() {
    if (kP.get() == lastkP && kD.get() == lastkD && kS.get() == lastkS) {
      return;
    }

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = kP.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();

    lastkP = kP.get();
    lastkD = kD.get();
    lastkS = kS.get();

    hoodMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void periodic() {
    log();
    updateGains();

    // tuningCommand =
    //     setPositionCommand(Rotation2d.fromDegrees(tuningTarget.get()))
    //         .until(() -> tuningTarget.get() == 0);

    // if (tuningTarget.get() != 0) {
    //   isTuning = true;
    //   CommandScheduler.getInstance().schedule(tuningCommand);
    // }

    // if (tuningTarget.get() == 0) {
    //   isTuning = false;
    // }
  }

  public void clearCache() {
    position = null;
  }

  private Rotation2d simPos = Rotation2d.kZero;

  public void setSimPosition(Rotation2d pos) {
    this.simPos = pos;
  }

  public Rotation2d getSimPos() {
    return this.simPos;
  }

  @Override
  public void simulationPeriodic() {
    ShotParameters params = ShotCalculator.getInstance().calculateParameters();

    setSimPosition(params.hoodAngle());
  }
}
