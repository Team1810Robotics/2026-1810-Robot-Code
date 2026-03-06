package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShotCalculator;

public class TurretSubsystem extends SubsystemBase {

  private final TalonFX turretMotor;
  private final DutyCycleEncoder turretEncoder;

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private double lastEncoderRaw = 0;
  private double unwrappedEncoder = 0;

  private int resetMotorPos = 0;

  private Rotation2d target;

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

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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

    target = Rotation2d.kZero;

    seedMotorFromAbsolute();

    setDefaultCommand(setFieldRelativeAngleCommand());
  }

  public Rotation2d getEncoderPosition() {

    double raw = turretEncoder.get();
    double delta = raw - lastEncoderRaw;

    if (delta > 0.5) {
      delta -= 1.0;
    } else if (delta < -0.5) {
      delta += 1.0;
    }

    unwrappedEncoder += delta;
    lastEncoderRaw = raw;

    double turretRot = -(unwrappedEncoder - TurretConstants.ENCODER_OFFSET);
    double turretRad = Rotations.of(turretRot).in(Radians);

    return Rotation2d.fromRadians(turretRad);
  }

  public void seedMotorFromAbsolute() {
    turretMotor.setPosition(getEncoderPosition().getRotations());
  }

  public Rotation2d getTurretAngle() {
    return Rotation2d.fromRotations(turretMotor.getPosition().getValueAsDouble());
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

  // FIX: ShotCalculator now returns a field-relative angle. Subtract robot rotation here
  // to convert to robot-relative, then convert to turret frame via robotRelativeToTurret().
  // Previously, ShotCalculator was already subtracting robot rotation, so this method was
  // double-subtracting and then negating — mirroring the target across the robot centerline.
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
        "Turret/Motor Position (robot relative)",
        getTurretAngleRobotRelative().getDegrees(),
        Degrees);

    DogLog.log("Turret/Encoder Position", getEncoderPosition().getDegrees(), Degrees);

    DogLog.log("Turret/Raw Encoder", turretEncoder.get(), Rotations);

    DogLog.log("Turret/Volts", turretMotor.getMotorVoltage().getValueAsDouble(), Volts);

    DogLog.log(
        "Turret/Motor Velocity", turretMotor.getVelocity().getValueAsDouble(), RotationsPerSecond);

    DogLog.log("Turret/Pose", getTurretPose());

    DogLog.log("Turret/Unwrapped Encoder", unwrappedEncoder);
  }

  @Override
  public void periodic() {
    if (resetMotorPos > 20) {
      turretMotor.setPosition(Rotations.of(getEncoderPosition().getRotations()));
      resetMotorPos = 0;
    }
    log();
    resetMotorPos++;
  }

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

    DogLog.log("Turret/SimPose", turretPose);
  }
}
