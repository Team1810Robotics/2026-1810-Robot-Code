package frc.robot.subsystems.indexer.spindexer;

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
import frc.robot.subsystems.indexer.spindexer.SpindexerConstants.SpindexerState;

public class SpindexerSubsystem extends SubsystemBase {
  private final SparkMax spinMotor;

  private SpindexerState state = SpindexerState.STOP;

  public SpindexerSubsystem() {
    spinMotor = new SparkMax(SpindexerConstants.SPIN_MOTOR, MotorType.kBrushless);

    SparkMaxConfig spinMotorConfig = new SparkMaxConfig();
    spinMotorConfig.idleMode(IdleMode.kCoast);
    spinMotorConfig.smartCurrentLimit(40);
    spinMotorConfig.inverted(true);

    spinMotor.configure(
        spinMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    state = SpindexerState.STOP;
  }

  public void spindex(SpindexerState state) {
    this.state = state;

    if (state == SpindexerState.STOP) {
      stop();
      return;
    }

    spinMotor.set(state.getPower());
  }

  public Command spinCommand(SpindexerState state) {
    return Commands.startEnd(() -> spindex(state), () -> stop(), this);
  }

  public void stop() {
    spinMotor.stopMotor();
  }

  public void log() {
    DogLog.log("Spindexer/State", state);
  }

  @Override
  public void periodic() {
    log();
  }
}
