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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.deploy.DeployConstants.deployState;

public class DeploySubsystem extends SubsystemBase {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final DutyCycleEncoder encoder;

  private final PIDController intakePIDController;
  private final ArmFeedforward feedforward;

  private deployState deployState;

  private double deployTarget = 0;

  private double lastEncoderRaw = 0.0;
  private double unwrappedEncoder = 0.0;

  public final DoubleSubscriber kP = DogLog.tunable("Intake/kP", 4.0);
  public final DoubleSubscriber kD = DogLog.tunable("Intake/kD", 0.25);
  public final DoubleSubscriber kS = DogLog.tunable("Intake/kS", 0.75);
  public final DoubleSubscriber kG = DogLog.tunable("Intake/kG", 0.461);

  public double lastkP, lastkD, lastkS, lastkG;

  public final DoubleSubscriber target = DogLog.tunable("Intake/DeployTarget", 0.0);

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
      deployState = DeployConstants.deployState.RETRACT;
      deployTarget = deployState.getPosition();

      unwrappedEncoder = 1;
    } else {
      deployState = DeployConstants.deployState.DEPLOY;
      deployTarget = deployState.getPosition();
    }
  }

  public void deploy(deployState state) {
    this.deployState = state;
    this.deployTarget = state.getPosition();
  }

  public Command deployCommand(deployState state) {
    return Commands.run(() -> deploy(state), this);
  }

  public Command agitateCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> deploy(DeployConstants.deployState.AGITATE), this),
            Commands.waitUntil(this::atSetpoint),
            Commands.runOnce(() -> deploy(DeployConstants.deployState.DEPLOY), this),
            Commands.waitUntil(this::atSetpoint))
        .finallyDo(interrupted -> stopDeploy());
  }

  public Command delpoyCommandNoRequirements(deployState state) {
    return Commands.runOnce(() -> deploy(state));
  }

  public void stopDeploy() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public Rotation2d getPosition() {
    double raw = encoder.get();
    double delta = raw - lastEncoderRaw;

    if (delta > 0.5) {
      delta -= 1.0;
    } else if (delta < -0.5) {
      delta += 1.0;
    }

    unwrappedEncoder += delta;
    lastEncoderRaw = raw;

    double armRotations =
        (unwrappedEncoder / DeployConstants.GEAR_RATIO) - DeployConstants.ENCODER_OFFSET;

    if (armRotations < 0) {
      armRotations += 1.0;
    }

    double armRadians = Units.rotationsToRadians(armRotations);
    return Rotation2d.fromRadians(armRadians);
  }

  public boolean atSetpoint() {
    return Math.abs(getPosition().getRadians() - deployTarget) < Degrees.of(15).in(Radians);
  }

  public void log() {
    DogLog.log("Intake/Deploy/Encoder/Connected", encoder.isConnected());
    DogLog.log("Intake/Deploy/Encoder/Raw Position", encoder.get(), Rotations);
    DogLog.log("Intake/Deploy/Encoder/Position", getPosition().getDegrees(), Degrees);

    DogLog.log("Intake/Deploy/State", deployState.name());

    DogLog.log("Intake/Deploy/Target", deployTarget, Degrees);

    DogLog.log("Intake/At Setpoint", atSetpoint());
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
    if (encoder.isConnected()) {
      double output = intakePIDController.calculate(getPosition().getRadians(), deployTarget);

      double feedforwardOut = feedforward.calculateWithVelocities(getPosition().getRadians(), 0, 0);

      double totalOutput = output + feedforwardOut;
      DogLog.log("Intake/Deploy/Output Voltage", totalOutput); // add this
      leftMotor.setVoltage(totalOutput);
    }

    updateGains();
    log();
  }
}
