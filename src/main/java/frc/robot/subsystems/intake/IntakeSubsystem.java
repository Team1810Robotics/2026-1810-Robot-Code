package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.intake.IntakeConstants.deployState;
import frc.robot.subsystems.intake.IntakeConstants.rollerState;

public class IntakeSubsystem extends SubsystemBase {
  private SparkMax leftMotor;
  private SparkMax rightMotor;
  private SparkMax rollerMotor;
  private DutyCycleEncoder leftEncoder;
  private DutyCycleEncoder rightEncoder;
  private final PIDController intakePIDController;

  private IntakeState state;

  private rollerState rollerState;
  private deployState deployState;

  public IntakeSubsystem() {
    rollerMotor = new SparkMax(IntakeConstants.ROLLER_MOTOR, MotorType.kBrushless);
    leftMotor = new SparkMax(IntakeConstants.LEFT_MOTOR, MotorType.kBrushless);
    rightMotor = new SparkMax(IntakeConstants.RIGHT_MOTOR, MotorType.kBrushless);

    leftEncoder = new DutyCycleEncoder(IntakeConstants.LEFT_ENCODER_ID);
    rightEncoder = new DutyCycleEncoder(IntakeConstants.RIGHT_ENCODER_ID);

    SparkMaxConfig rightMotorconfig = new SparkMaxConfig();
    rightMotorconfig.follow(IntakeConstants.LEFT_MOTOR, true);
    rightMotorconfig.idleMode(IdleMode.kCoast);
    rightMotorconfig.smartCurrentLimit(40);

    rightMotor.configure(
        rightMotorconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    leftMotorConfig.idleMode(IdleMode.kCoast);
    leftMotorConfig.smartCurrentLimit(40);

    leftMotor.configure(
        leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    rollerMotorConfig.idleMode(IdleMode.kCoast);
    rollerMotorConfig.smartCurrentLimit(40);

    rollerMotor.configure(
        rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakePIDController =
        new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
  }

  public void deploy(deployState state) {
    this.deployState = state;

    if (leftEncoder.isConnected() && rightEncoder.isConnected()) {
      double output = intakePIDController.calculate(getLeftMeasurement(), state.getPosition());
      leftMotor.setVoltage(output);
    } else {
      fullStop();
    }
  }

  public Command deployCommand(deployState state) {
    return Commands.startEnd(() -> deploy(state), () -> stopDeploy(), this);
  }

  public void roller(rollerState state) {
    this.rollerState = state;
    if (state == rollerState.STOP) {
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

  public double getLeftMeasurement() {
    double position = leftEncoder.get() - IntakeConstants.LEFT_ENCODER_OFFSET;
    double degrees = Units.rotationsToDegrees(position);

    return degrees;
  }

  public double getRightMeasurement() {
    double position = rightEncoder.get() - IntakeConstants.RIGHT_ENCODER_OFFSET;
    double degrees = Units.rotationsToDegrees(position);

    return degrees;
  }

  public void log() {
    DogLog.log("Intake/LeftEncoder/Connected", leftEncoder.isConnected());
    DogLog.log("Intake/LeftEncoder/Position", getLeftMeasurement());

    DogLog.log("Intake/RightEncoder/Connected", rightEncoder.isConnected());
    DogLog.log("Intake/RightEncoder/Position", getRightMeasurement());

    DogLog.log("Intake/RollerState", rollerState);
    DogLog.log("Intake/DeployState", deployState);
    DogLog.log("Intake/IntakeState", state);
  }
}
