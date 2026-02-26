package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerConstants.indexerState;

public class IndexerSubsystem extends SubsystemBase {
  private final SparkMax spinMotor;
  private final SparkMax kickerMotor;

  private indexerState state;

  public IndexerSubsystem() {
    spinMotor = new SparkMax(IndexerConstants.SPIN_MOTOR, MotorType.kBrushless);
    kickerMotor = new SparkMax(IndexerConstants.KICKER_MOTOR, MotorType.kBrushless);

    SparkMaxConfig spinMotorConfig = new SparkMaxConfig();
    spinMotorConfig.idleMode(IdleMode.kCoast);
    spinMotorConfig.smartCurrentLimit(40);
    spinMotorConfig.inverted(true);

    spinMotor.configure(
        spinMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig kickerMotorConfig = new SparkMaxConfig();
    kickerMotorConfig.idleMode(IdleMode.kCoast);
    kickerMotorConfig.smartCurrentLimit(60);
    kickerMotorConfig.inverted(true);

    kickerMotor.configure(
        kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    state = indexerState.STOP;
  }

  public void index(indexerState state) {
    this.state = state;

    if (state == indexerState.STOP) {
      fullStop();
      return;
    }

    spinMotor.set(state.getSpinPower());
    kickerMotor.set(state.getKickPower());
  }

  public Command indexCommand(indexerState state) {
    return Commands.startEnd(() -> index(state), () -> fullStop(), this);
  }

  public Command kickCommand(indexerState state) {
    return Commands.startEnd(
        () -> kickerMotor.set(state.getKickPower()), () -> kickerMotor.stopMotor(), this);
  }

  public Command spinCommand(indexerState state) {
    return Commands.startEnd(
        () -> spinMotor.set(state.getSpinPower()), () -> spinMotor.stopMotor(), this);
  }

  public void fullStop() {
    stopSpindexer();
    stopKicker();
  }

  public void stopSpindexer() {
    spinMotor.stopMotor();
  }

  public void stopKicker() {
    kickerMotor.stopMotor();
  }

  public void log() {
    DogLog.log("Indexer/State", state.name());
  }

  @Override
  public void periodic() {
    log();
  }
}
