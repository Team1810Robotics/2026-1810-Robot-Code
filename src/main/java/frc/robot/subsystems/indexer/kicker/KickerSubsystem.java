package frc.robot.subsystems.indexer.kicker;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.kicker.KickerConstants.KickerState;

public class KickerSubsystem extends SubsystemBase {
  private final TalonFX kickerMotor;

  private KickerState state;

  public KickerSubsystem() {
    this.kickerMotor = new TalonFX(KickerConstants.KICKER_MOTOR);

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 80;

    kickerMotor.getConfigurator().apply(cfg);
  }

  public void kick(KickerState state) {
    this.state = state;

    if (state == KickerState.STOP) {
      stop();
      return;
    }

    kickerMotor.set(state.getPower());
  }

  public Command kickCommand(KickerState state) {
    return Commands.startEnd(() -> kick(state), () -> stop(), this);
  }

  public void stop() {
    kickerMotor.stopMotor();
  }

  public void log() {
    DogLog.log("Kicker/State", state);
  }

  @Override
  public void periodic() {
    log();
  }
}
