package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

  private final TalonFX turretMotor;

  private final DutyCycleEncoder turretEncoder;

  private final PositionVoltage positionRequest = new PositionVoltage(0);

  public final DoubleSubscriber kP = DogLog.tunable("Turret/Gains/kP", 0.0);
  public final DoubleSubscriber kI = DogLog.tunable("Turret/Gains/kI", 0.0);
  public final DoubleSubscriber kD = DogLog.tunable("Turret/Gains/kD", 0.0);

  public final DoubleSubscriber kS = DogLog.tunable("Turret/Gains/kS", 0.0);
  public final DoubleSubscriber kV = DogLog.tunable("Turret/Gains/kV", 0.0);
  public final DoubleSubscriber kA = DogLog.tunable("Turret/Gains/kA", 0.0);

  private static TurretSubsystem instance;

  private TurretSubsystem() {
    turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = TurretConstants.GEAR_RATIO;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.MAX_ANGLE.in(Degrees);

    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.MIN_ANGLE.in(Degrees);

    turretMotor.getConfigurator().apply(config);

    turretEncoder = new DutyCycleEncoder(TurretConstants.TURRET_ENCODER_ID);

    seedMotorFromAbsolute();
  }

  public Rotation2d getEncoderPosition() {
    return Rotation2d.fromRotations(turretEncoder.get());
  }

  public void seedMotorFromAbsolute() {
    turretMotor.setPosition(getEncoderPosition().getRotations());
  }

  public Rotation2d getTurretAngle() {
    return Rotation2d.fromRotations(turretMotor.getPosition().getValueAsDouble());
  }

  public void setTurretAngle(Rotation2d targetAngle) {
    double rotations =
        MathUtil.clamp(
            targetAngle.getRotations(),
            TurretConstants.MIN_ANGLE.in(Rotations),
            TurretConstants.MAX_ANGLE.in(Rotations));

    turretMotor.setControl(positionRequest.withPosition(rotations));
  }

  public void updateGains() {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kA = kA.get();

    turretMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void periodic() {
    updateGains();
  }

  public static TurretSubsystem getInstance() {
    if (instance == null) {
      instance = new TurretSubsystem();
    }
    return instance;
  }
}
