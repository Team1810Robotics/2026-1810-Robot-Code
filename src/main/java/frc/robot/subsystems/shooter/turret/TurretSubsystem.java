package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.TunablePIDF;
import frc.robot.util.TunablePIDF.TunablePIDFGains;

public class TurretSubsystem extends SubsystemBase {

  private final TalonFX turretMotor;
  private final DutyCycleEncoder turretEncoder;

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private double lastEncoderRaw = 0;
  private double unwrappedEncoder = 0;

  private int resetMotorPos = 0;

  private Rotation2d target;
  private Rotation2d motorPosition;
  private Rotation2d encoderPosition;

  private final TunablePIDF tunablePIDF;

  private final DoubleSubscriber positionTarget = DogLog.tunable("Turret/Position", 0.0);

  // private final DoubleSubscriber veloTarget = DogLog.tunable("Turret/Velocity", 0.0);
  // private final BooleanSubscriber isVelocity = DogLog.tunable("Turret/Should Tune Velocity",
  // false);

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

    tunablePIDF = new TunablePIDF("Turret");

    turretMotor.getConfigurator().apply(config);

    turretEncoder = new DutyCycleEncoder(TurretConstants.TURRET_ENCODER_ID);
    lastEncoderRaw = turretEncoder.get();
    unwrappedEncoder = lastEncoderRaw;
    seedMotorFromAbsolute();

    setDefaultCommand(setFieldRelativeAngleCommand());
  }

  private void updateEncoderUnwrap() {
    double raw = turretEncoder.get();
    double delta = raw - lastEncoderRaw;

    if (delta > 0.5) {
      delta -= 1.0;
    } else if (delta < -0.5) {
      delta += 1.0;
    }

    unwrappedEncoder += delta;
    lastEncoderRaw = raw;
  }

  public Rotation2d getEncoderPosition() {
    if (encoderPosition != null) {
      return encoderPosition;
    }

    double turretRot = -(unwrappedEncoder - TurretConstants.ENCODER_OFFSET);
    double turretRad = Rotations.of(turretRot).in(Radians);

    encoderPosition = Rotation2d.fromRadians(turretRad);
    return encoderPosition;
  }

  public void seedMotorFromAbsolute() {
    turretMotor.setPosition(getEncoderPosition().getRotations());
  }

  public Rotation2d getTurretAngle() {
    if (motorPosition != null) {
      return motorPosition;
    }

    motorPosition = Rotation2d.fromRotations(turretMotor.getPosition().getValueAsDouble());
    return motorPosition;
  }

  public Rotation2d getTurretAngleRobotRelative() {
    return turretToRobotRelative(getTurretAngle());
  }

  public Rotation2d turretToRobotRelative(Rotation2d turretAngle) {
    return turretAngle.minus(Rotation2d.fromDegrees(TurretConstants.ROBOT_RELATIVE_OFFSET_DEG));
  }

  private Rotation2d robotRelativeToTurret(Rotation2d rrAngle) {
    Rotation2d ang =
        rrAngle.plus(Rotation2d.fromDegrees(TurretConstants.ROBOT_RELATIVE_OFFSET_DEG));

    DogLog.log("Turret/FieldRelative/TurretFrame", ang.getDegrees());

    return ang;
  }

  public void setTurretAngle(Rotation2d targetAngle) {
    double rotations =
        MathUtil.clamp(
            targetAngle.getRotations(),
            TurretConstants.MIN_ANGLE.in(Rotations),
            TurretConstants.MAX_ANGLE.in(Rotations));

    DogLog.log("Turret/FieldRelative/Applied", Rotations.of(rotations).in(Degrees));

    target = Rotation2d.fromRotations(rotations);

    turretMotor.setControl(motionMagicRequest.withPosition(rotations));
  }

  public void setTurretVelocity(double rotationsPerSecond) {
    turretMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
  }

  public boolean atTargetAngle() {
    return Math.abs(target.getDegrees() - getTurretAngle().getDegrees()) < 30;
  }

  public void setRobotRelativeAngle(Rotation2d rrAngle) {
    setTurretAngle(robotRelativeToTurret(rrAngle));
  }

  public Command setTurretAngleCommand(Rotation2d targetAngle) {
    return Commands.startEnd(() -> setTurretAngle(targetAngle), () -> stop(), this);
  }

  public Command setRobotRelativeAngleCommand(Rotation2d rrAngle) {
    return Commands.startEnd(() -> setRobotRelativeAngle(rrAngle), () -> stop(), this);
  }

  public void setFieldRelativeAngle() {
    Rotation2d fieldRelativeAngle =
        ShotCalculator.getInstance().calculateParameters().turretAngle();
    Rotation2d robotRelativeAngle =
        fieldRelativeAngle.minus(RobotContainer.getDrivetrain().getPose().getRotation());

    DogLog.log("Turret/FieldRelative/Calculated RR Angle", robotRelativeAngle.getDegrees());

    setRobotRelativeAngle(robotRelativeAngle);
  }

  public Command setFieldRelativeAngleCommand() {
    return Commands.run(() -> setFieldRelativeAngle(), this).andThen(() -> stop());
  }

  public Pose3d getTurretPose() {

    Pose3d botPose = new Pose3d(RobotContainer.getDrivetrain().getPose());

    Rotation2d rrAngle = turretToRobotRelative(getTurretAngle());

    return new Pose3d(
        botPose.transformBy(TurretConstants.ROBOT_TO_TURRET).getTranslation(),
        new Rotation3d(0, 0, rrAngle.getRadians()));
  }

  public void stop() {
    turretMotor.stopMotor();
  }

  public void log() {
    DogLog.log("Turret/Motor Position (turret frame)", getTurretAngle().getDegrees(), Degrees);
    DogLog.log(
        "Turret/Motor Position (robot frame)", getTurretAngleRobotRelative().getDegrees(), Degrees);

    DogLog.log("Turret/Encoder Position", getEncoderPosition().getDegrees(), Degrees);
    DogLog.log("Turret/Raw Encoder Position", turretEncoder.get(), Rotations);

    DogLog.log("Turret/Volts", turretMotor.getMotorVoltage().getValueAsDouble(), Volts);

    DogLog.log("Turret/Velocity", turretMotor.getVelocity().getValueAsDouble(), RotationsPerSecond);
  }

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

  @Override
  public void periodic() {
    updateGains();

    updateEncoderUnwrap(); // always runs, keeps lastEncoderRaw current

    if (resetMotorPos > 50) {
      turretMotor.setPosition(Rotations.of(getEncoderPosition().getRotations()));
      resetMotorPos = 0;
    }
    log();
    resetMotorPos++;

    // turretMotor.setControl(
    //     motionMagicRequest.withPosition(Degrees.of(positionTarget.get()).in(Rotations)));
  }

  public Pose3d simPose = new Pose3d();

  @Override
  public void simulationPeriodic() {

    Rotation2d fieldRelativeAngle =
        ShotCalculator.getInstance().calculateParameters().turretAngle();
    Rotation2d robotRelativeAngle =
        fieldRelativeAngle.minus(RobotContainer.getDrivetrain().getPose().getRotation());

    Pose3d turretPose =
        new Pose3d(
            TurretConstants.ROBOT_TO_TURRET.getTranslation(),
            new Rotation3d(0, 0, robotRelativeAngle.getRadians()));

    simPose = turretPose;
  }

  public void clearCache() {
    motorPosition = null;
    encoderPosition = null;
  }
}
