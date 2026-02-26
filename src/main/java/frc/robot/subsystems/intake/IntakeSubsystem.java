package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.intake.IntakeConstants.deployState;
import frc.robot.subsystems.intake.IntakeConstants.rollerState;

public class IntakeSubsystem extends SubsystemBase {
  private SparkMax leftMotor;
  private SparkMax rightMotor;
  private SparkMax rollerMotor;
  private DutyCycleEncoder encoder;

  private final PIDController intakePIDController;
  private final ArmFeedforward feedforward;

  private IntakeState state;

  private rollerState rollerState;
  private deployState deployState;

  private double deployTarget = 0;

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

    feedforward = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, 0);

    deployState = IntakeConstants.deployState.RETRACT;
    rollerState = IntakeConstants.rollerState.STOP;
  }

  public void deploy(deployState state) {
    this.deployState = state;
    this.deployTarget = state.getPosition();

    if (encoder.isConnected()) {
      double output =
          intakePIDController.calculate(getPosition().getRadians(), state.getPosition());
      double feedforwardOut = feedforward.calculateWithVelocities(getPosition().getRadians(), 0, 0);

      leftMotor.setVoltage(output + feedforwardOut);
    } else {
      fullStop();
    }
  }

  public Command deployCommand(deployState state) {
    return Commands.run(() -> deploy(state), this).finallyDo(() -> stopDeploy());
  }

  public Command deployCommand() {
    if (deployState == IntakeConstants.deployState.DEPLOY) {
      return deployCommand(IntakeConstants.deployState.RETRACT);
    } else if (deployState == IntakeConstants.deployState.RETRACT) {
      return deployCommand(IntakeConstants.deployState.DEPLOY);
    } else {
      return new InstantCommand();
    }
  }

  public void roller(rollerState state) {
    this.rollerState = state;
    if (state == IntakeConstants.rollerState.STOP) {
      stopRoller();
      return;
    }

    rollerMotor.set(state.getPower());
  }

  public Command rollerCommand(rollerState state) {
    return Commands.startEnd(() -> roller(state), () -> stopRoller(), this);
  }

  public void setIntakeState(IntakeState state) {
    this.state = state;

    roller(state.getRollerState());
    deploy(state.getDeployState());
  }

  public Command setIntakeStateCommand(IntakeState state) {
    return Commands.run(() -> setIntakeState(state), this).finallyDo(() -> fullStop());
  }

  public void set(double pow) {
    leftMotor.set(pow);
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

  // Add these fields to your IntakeSubsystem
  private double lastEncoderRaw = 0.0;
  private double unwrappedEncoder = 0.0;

  public Rotation2d getPosition() {
    // 1️⃣ Read the raw encoder value (0–1)
    double raw = encoder.get();

    // 2️⃣ Compute delta since last reading
    double delta = raw - lastEncoderRaw;

    // 3️⃣ Handle wrap-around
    if (delta > 0.5) { // jumped from 1 -> 0
      delta -= 1.0;
    } else if (delta < -0.5) { // jumped from 0 -> 1
      delta += 1.0;
    }

    // 4️⃣ Update continuous unwrapped rotation
    unwrappedEncoder += delta;
    lastEncoderRaw = raw;

    // 5️⃣ Convert unwrapped encoder rotations → arm rotations
    double armRotations =
        (unwrappedEncoder / IntakeConstants.GEAR_RATIO) - IntakeConstants.ENCODER_OFFSET;

    // 6️⃣ Optional: wrap if your arm motion < 1 rotation
    if (armRotations < 0) {
      armRotations += 1.0;
    }

    // 7️⃣ Convert arm rotations to radians for WPILib
    double armRadians = Units.rotationsToRadians(armRotations);

    return Rotation2d.fromRadians(armRadians);
  }

  public void log() {
    DogLog.log("Intake/Encoder/Connected", encoder.isConnected());
    DogLog.log("Intake/Encoder/Raw Position", encoder.get());
    DogLog.log("Intake/Encoder/Position", getPosition().getDegrees(), Degrees);

    DogLog.log("Intake/RollerState", rollerState);
    DogLog.log("Intake/DeployState", deployState);
    DogLog.log("Intake/IntakeState", state);

    DogLog.log("Intake/DeployTarget", deployTarget, Degrees);
  }

  @Override
  public void periodic() {
    log();
  }
}
