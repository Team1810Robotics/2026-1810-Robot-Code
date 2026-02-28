package frc.robot.subsystems.intake;

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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.intake.IntakeConstants.deployState;
import frc.robot.subsystems.intake.IntakeConstants.rollerState;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final SparkMax rollerMotor;
  private final DutyCycleEncoder encoder;

  private final PIDController intakePIDController;
  private final ArmFeedforward feedforward;

  private IntakeState state;
  private rollerState rollerState;
  private deployState deployState;

  private double deployTarget = 0;

  private double lastEncoderRaw = 0.0;
  private double unwrappedEncoder = 0.0;

  public IntakeSubsystem() {
    rollerMotor = new SparkMax(IntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
    leftMotor = new SparkMax(IntakeConstants.LEFT_DELPOY_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(IntakeConstants.RIGHT_DEPLOY_MOTOR_ID, MotorType.kBrushless);

    encoder = new DutyCycleEncoder(IntakeConstants.LEFT_ENCODER_ID);

    SparkMaxConfig rightMotorconfig = new SparkMaxConfig();
    rightMotorconfig.follow(IntakeConstants.LEFT_DELPOY_MOTOR_ID, true);
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

    SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    rollerMotorConfig.idleMode(IdleMode.kCoast);
    rollerMotorConfig.smartCurrentLimit(40);
    rollerMotorConfig.inverted(true);
    rollerMotor.configure(
        rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakePIDController = new PIDController(IntakeConstants.kP, 0, IntakeConstants.kD);
    intakePIDController.setTolerance(Degrees.of(10).in(Radians));

    feedforward = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, 0);

    deployState = IntakeConstants.deployState.RETRACT;
    rollerState = IntakeConstants.rollerState.STOP;
    state = IntakeConstants.IntakeState.OUT_INTAKE;

    deployTarget = deployState.getPosition();
  }

  public void deploy(deployState state) {
    this.deployState = state;
    this.deployTarget = state.getPosition();
  }

  public Command deployCommand(deployState state) {
    return Commands.runOnce(() -> deploy(state), this);
  }

  public Command delpoyCommandNoRequirements(deployState state) {
    return Commands.runOnce(() -> deploy(state));
  }

  public void roller(rollerState state) {
    this.rollerState = state;
    if (state == IntakeConstants.rollerState.STOP) {
      rollerMotor.stopMotor();
      return;
    }

    rollerMotor.set(state.getPower());
  }

  public Command rollerCommand(rollerState state) {
    return Commands.startEnd(() -> roller(state), () -> rollerMotor.stopMotor(), this);
  }

  public void setIntakeState(IntakeState state) {
    this.state = state;
    roller(state.getRollerState());
    deploy(state.getDeployState());
  }

  public Command setIntakeStateCommand(IntakeState state) {
    return Commands.runOnce(() -> setIntakeState(state), this);
  }

  public Command advanceState() {
    return Commands.runOnce(
        () -> {
          IntakeState next;
          switch (state) {
            case OUT_INTAKE:
              next = IntakeState.RETRACT;
              break;
            case RETRACT:
              next = IntakeState.OUT_INTAKE;
              break;
            default:
              return;
          }
          setIntakeState(next);
        },
        this);
  }

  public void stopRoller() {
    rollerMotor.stopMotor();
  }

  public void stopDeploy() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void fullStop() {
    stopDeploy();
    stopRoller();
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
        (unwrappedEncoder / IntakeConstants.GEAR_RATIO) - IntakeConstants.ENCODER_OFFSET;

    if (armRotations < 0) {
      armRotations += 1.0;
    }

    double armRadians = Units.rotationsToRadians(armRotations);
    return Rotation2d.fromRadians(armRadians);
  }

  public void log() {
    DogLog.log("Intake/Encoder/Connected", encoder.isConnected());
    DogLog.log("Intake/Encoder/Raw Position", encoder.get(), Rotations);
    DogLog.log("Intake/Encoder/Position", getPosition().getDegrees(), Degrees);
    DogLog.log("Intake/RollerState", rollerState.name());
    DogLog.log("Intake/DeployState", deployState.name());
    DogLog.log("Intake/IntakeState", state.name());
    DogLog.log("Intake/DeployTarget", deployTarget, Degrees);
  }

  @Override
  public void periodic() {
    if (encoder.isConnected()) {
      double output = intakePIDController.calculate(getPosition().getRadians(), deployTarget);

      double feedforwardOut = feedforward.calculateWithVelocities(getPosition().getRadians(), 0, 0);

      leftMotor.setVoltage(output + feedforwardOut);
    } else {
      stopDeploy();
    }

    log();
  }
}
