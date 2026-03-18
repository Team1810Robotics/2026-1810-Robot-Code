package frc.robot.subsystems.intake.deploy;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.deploy.DeployConstants.DeployState;

public class DeploySubsystem extends SubsystemBase {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final DutyCycleEncoder encoder;

  private final PIDController intakePIDController;
  private final ArmFeedforward feedforward;

  private DeployState deployState;

  private Rotation2d deployTarget;
  private Rotation2d position;

  private double lastEncoderRaw = 0.0;
  private double unwrappedEncoder = 0.0;

  public final DoubleSubscriber kP = DogLog.tunable("Intake/kP", 4.0);
  public final DoubleSubscriber kD = DogLog.tunable("Intake/kD", 0.25);
  public final DoubleSubscriber kS = DogLog.tunable("Intake/kS", 0.75);
  public final DoubleSubscriber kG = DogLog.tunable("Intake/kG", 0.461);

  public double lastkP, lastkD, lastkS, lastkG;

  public final DoubleSubscriber targetSubscriber = DogLog.tunable("Intake/DeployTarget", 0.0);

  public DeploySubsystem() {
    leftMotor = new SparkMax(DeployConstants.LEFT_DELPOY_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(DeployConstants.RIGHT_DEPLOY_MOTOR_ID, MotorType.kBrushless);

    encoder = new DutyCycleEncoder(DeployConstants.LEFT_ENCODER_ID);

    SparkMaxConfig rightMotorconfig = new SparkMaxConfig();
    rightMotorconfig.follow(DeployConstants.LEFT_DELPOY_MOTOR_ID, true);
    rightMotorconfig.idleMode(IdleMode.kCoast);
    rightMotorconfig.smartCurrentLimit(40);
    rightMotor.configure(
        rightMotorconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    leftMotorConfig.idleMode(IdleMode.kCoast);
    leftMotorConfig.smartCurrentLimit(40);
    leftMotorConfig.inverted(true);
    leftMotor.configure(
        leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakePIDController = new PIDController(DeployConstants.kP, 0, DeployConstants.kD);
    intakePIDController.setTolerance(Degrees.of(10).in(Radians));

    feedforward = new ArmFeedforward(DeployConstants.kS, DeployConstants.kG, 0);

    if (encoder.get() < .4) {
      deployState = DeployConstants.DeployState.RETRACT;
      deployTarget = deployState.getPosition();

      unwrappedEncoder = 1;
    } else {
      deployState = DeployConstants.DeployState.DEPLOY;
      deployTarget = deployState.getPosition();
    }
  }

  public void setState(DeployState state) {
    this.deployState = state;
    this.deployTarget = state.getPosition();
  }

  public DeployState getState() {
    return deployState;
  }

  public Command deployCommand(DeployState state) {
    return Commands.run(() -> setState(state), this);
  }

  public Command agitateCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> setState(DeployConstants.DeployState.AGITATE), this),
            Commands.waitUntil(this::atSetpoint),
            Commands.runOnce(() -> setState(DeployConstants.DeployState.DEPLOY), this),
            Commands.waitUntil(this::atSetpoint))
        .finallyDo(interrupted -> stopDeploy());
  }

  public Command delpoyCommandNoRequirements(DeployState state) {
    return Commands.runOnce(() -> setState(state));
  }

  public void stopDeploy() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  private void updateEncoderUnwrap() {
    double raw = encoder.get();
    double delta = raw - lastEncoderRaw;

    if (delta > 0.5) {
      delta -= 1.0;
    } else if (delta < -0.5) {
      delta += 1.0;
    }

    unwrappedEncoder += delta;
    lastEncoderRaw = raw;
  }

  public Rotation2d getPosition() {
    if (position != null) {
      return position;
    }

    double armRotations =
        (unwrappedEncoder / DeployConstants.GEAR_RATIO) - DeployConstants.ENCODER_OFFSET;

    if (armRotations < 0) {
      armRotations += 1.0;
    }

    position = Rotation2d.fromRotations(armRotations);
    return position;
  }

  public boolean atSetpoint() {
    return Math.abs(getPosition().getRadians() - deployTarget.getRadians())
        < Degrees.of(15).in(Radians);
  }

  public Rotation2d getTargetPosition() {
    return deployTarget;
  }

  public void log() {
    DogLog.log("Intake/Deploy/Position", getPosition().getDegrees(), Degrees);
    DogLog.log("Intake/Deploy/State", deployState.name());
    DogLog.log("Intake/Deploy/Target", deployTarget.getDegrees(), Degrees);
    DogLog.log("Intake/Deploy/Raw Encoder", encoder.get());
  }

  public void updateGains() {
    if (lastkP != kP.get() || lastkD != kD.get() || lastkS != kS.get() || lastkG != kG.get()) {
      intakePIDController.setPID(kP.get(), 0, kD.get());
      feedforward.setKg(kG.get());
      feedforward.setKs(kS.get());

      lastkP = kP.get();
      lastkD = kD.get();
      lastkS = kS.get();
      lastkG = kG.get();
    }
  }

  @Override
  public void periodic() {
    updateEncoderUnwrap(); // always runs, keeps lastEncoderRaw current

    if (encoder.isConnected()) {
      double pidOut =
          intakePIDController.calculate(getPosition().getRadians(), deployTarget.getRadians());
      double feedforwardOut = feedforward.calculateWithVelocities(getPosition().getRadians(), 0, 0);
      double totalOutput = pidOut + feedforwardOut;

      DogLog.log("Intake/Deploy/Output Voltage", totalOutput);

      leftMotor.setVoltage(totalOutput);
    }

    updateGains();
    log();
  }

  public void clearCache() {
    position = null;
  }
}
