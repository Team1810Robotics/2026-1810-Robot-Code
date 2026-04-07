package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.TunablePIDF;
import frc.robot.util.TunablePIDF.TunablePIDFGains;

public class TurretSubsystem extends SubsystemBase {

  // Hardware
  private final TalonFX turretMotor;
  private final DutyCycleEncoder turretEncoder;

  // Control requests
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

  // Encoder unwrapping state
  private double lastEncoderRaw = 0;
  private double unwrappedEncoder = 0;

  // Cached positions (cleared each loop via clearCache())
  private Rotation2d motorPosition;
  private Rotation2d encoderPosition;

  // Current clamped target in motor frame
  private Rotation2d motorTarget;

  private final TunablePIDF tunablePIDF;

  // private final DoubleSubscriber positionTarget = DogLog.tunable("Turret/Position", 0.0);

  public TurretSubsystem() {
    turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = TurretConstants.GEAR_RATIO;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.MAX_ANGLE.in(Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.MIN_ANGLE.in(Rotations);

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.kP = TurretConstants.kP;
    config.Slot0.kD = TurretConstants.kD;
    config.Slot0.kS = TurretConstants.kS;
    config.Slot0.kV = TurretConstants.kV;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = TurretConstants.MOTION_MAGIC_ACCELERATION;
    config.MotionMagic.MotionMagicJerk = TurretConstants.MOTION_MAGIC_JERK;

    turretMotor.getConfigurator().apply(config);

    turretEncoder = new DutyCycleEncoder(TurretConstants.TURRET_ENCODER_ID);
    lastEncoderRaw = turretEncoder.get();
    unwrappedEncoder = lastEncoderRaw;
    seedMotorFromAbsolute();

    tunablePIDF = new TunablePIDF("Turret");

    setDefaultCommand(setFieldRelativeAngleCommand());
  }

  public Rotation2d motorToField(Rotation2d motorAngle) {
    return motorAngle
        .minus(Rotation2d.fromDegrees(TurretConstants.ROBOT_RELATIVE_OFFSET_DEG))
        .plus(RobotContainer.getDrivetrain().getPose().getRotation());
  }

  public Rotation2d fieldToMotor(Rotation2d fieldAngle) {
    return fieldAngle
        .minus(RobotContainer.getDrivetrain().getPose().getRotation())
        .plus(Rotation2d.fromDegrees(TurretConstants.ROBOT_RELATIVE_OFFSET_DEG));
  }

  private void setMotorAngle(Rotation2d target, double velocityFFRotPerSec) {
    double rotations =
        MathUtil.clamp(
            target.getRotations(),
            TurretConstants.MIN_ANGLE.in(Rotations),
            TurretConstants.MAX_ANGLE.in(Rotations));

    motorTarget = Rotation2d.fromRotations(rotations);

    turretMotor.setControl(
        motionMagicRequest
            .withPosition(rotations)
            .withFeedForward(velocityFFRotPerSec * TurretConstants.kV));
  }

  public void setFieldRelativeAngle() {
    Rotation2d fieldTarget = ShotCalculator.getInstance().calculateParameters().turretAngle();

    // Robot omega in rot/s — negate because if the robot spins CW the turret
    // must spin CCW at the same rate to hold its field-relative heading
    double robotOmegaRotPerSec =
        RobotContainer.getDrivetrain().getRobotRelativeSpeeds().omegaRadiansPerSecond
            / (2 * Math.PI);

    setMotorAngle(fieldToMotor(fieldTarget), -robotOmegaRotPerSec);
  }

  public void setRobotRelativeAngle(Rotation2d robotRelativeAngle) {
    setMotorAngle(
        robotRelativeAngle.plus(Rotation2d.fromDegrees(TurretConstants.ROBOT_RELATIVE_OFFSET_DEG)),
        0.0);
  }

  public void set(double dutyCycle) {
    turretMotor.setControl(new DutyCycleOut(dutyCycle));
  }

  public void stop() {
    turretMotor.stopMotor();
  }

  public Command setFieldRelativeAngleCommand() {
    return Commands.run(this::setFieldRelativeAngle, this).finallyDo(this::stop);
  }

  public Command setRobotRelativeAngleCommand(Rotation2d angle) {
    return Commands.startEnd(() -> setRobotRelativeAngle(angle), this::stop, this);
  }

  public Rotation2d getMotorAngle() {
    if (motorPosition != null) {
      return motorPosition;
    }
    motorPosition = Rotation2d.fromRotations(turretMotor.getPosition().getValueAsDouble());
    return motorPosition;
  }

  public Rotation2d getRobotRelativeAngle() {
    return getMotorAngle().minus(Rotation2d.fromDegrees(TurretConstants.ROBOT_RELATIVE_OFFSET_DEG));
  }

  public Rotation2d getFieldAngle() {
    return motorToField(getMotorAngle());
  }

  public Rotation2d getEncoderPosition() {
    if (encoderPosition != null) {
      return encoderPosition;
    }
    double turretRot = -(unwrappedEncoder - TurretConstants.ENCODER_OFFSET);
    encoderPosition = Rotation2d.fromRadians(Rotations.of(turretRot).in(Radians));
    return encoderPosition;
  }

  public boolean atTargetAngle() {
    if (motorTarget == null) return false;
    return Math.abs(motorTarget.getDegrees() - getMotorAngle().getDegrees()) < 5;
  }

  public void seedMotorFromAbsolute() {
    turretMotor.setPosition(getEncoderPosition().getRotations());
  }

  private void updateEncoderUnwrap() {
    double raw = turretEncoder.get();
    double delta = raw - lastEncoderRaw;

    if (delta > 0.5) delta -= 1.0;
    else if (delta < -0.5) delta += 1.0;

    unwrappedEncoder += delta;
    lastEncoderRaw = raw;
  }

  // ---------------------------------------------------------------------------
  // Gains
  // ---------------------------------------------------------------------------

  public void updateGains() {
    TunablePIDFGains gains = tunablePIDF.getGains();
    if (!gains.hasChanged()) return;

    Slot0Configs cfg = new Slot0Configs();
    cfg.kP = gains.kP();
    cfg.kD = gains.kD();
    cfg.kS = gains.kS();
    cfg.kV = gains.kV();
    cfg.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    turretMotor.getConfigurator().apply(cfg);
  }

  public void log() {
    DogLog.log("Turret/Motor Angle (motor frame)", getMotorAngle().getDegrees(), Degrees);
    DogLog.log("Turret/Motor Angle (robot frame)", getRobotRelativeAngle().getDegrees(), Degrees);
    DogLog.log("Turret/Motor Angle (field frame)", getFieldAngle().getDegrees(), Degrees);
    DogLog.log("Turret/Encoder Position", getEncoderPosition().getDegrees(), Degrees);
    DogLog.log("Turret/Raw Encoder", turretEncoder.get(), Rotations);
    DogLog.log("Turret/Volts", turretMotor.getMotorVoltage().getValueAsDouble(), Volts);
    DogLog.log("Turret/Velocity", turretMotor.getVelocity().getValueAsDouble(), RotationsPerSecond);
    DogLog.log("Turret/At Setpoint", atTargetAngle());
  }

  @Override
  public void periodic() {
    updateEncoderUnwrap();
    log();
    // updateGains();

    if (Math.abs(turretMotor.getPosition().getValueAsDouble() - getEncoderPosition().getRotations())
        > Degrees.of(2.5).in(Rotations)) {
      seedMotorFromAbsolute();
    }
  }

  public void clearCache() {
    motorPosition = null;
    encoderPosition = null;
  }
}
