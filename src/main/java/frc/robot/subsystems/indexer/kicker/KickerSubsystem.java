package frc.robot.subsystems.indexer.kicker;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state.RobotState;
import frc.robot.subsystems.indexer.kicker.KickerConstants.KickerState;

public class KickerSubsystem extends SubsystemBase {
  private final TalonFX kickerMotor;

  // private final TunablePIDF tunablePIDF = new TunablePIDF("kicker");

  private final BooleanSubscriber tuningMode = DogLog.tunable("Kicker/Tuning Mode", false);
  private boolean isTuning = false;

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

  public void setState(KickerState state) {
    this.state = state;
  }

  public Command kickCommand(KickerState state) {
    return Commands.startEnd(() -> setState(state), () -> stop(), this);
  }

  public void stop() {
    kickerMotor.stopMotor();
  }

  public void log() {
    DogLog.log("Kicker/State", state);
  }

  // public void updateGains() {
  //   TunablePIDFGains gains = tunablePIDF.getGains();

  //   if (!gains.hasChanged()) return;

  //   Slot0Configs cfg = new Slot0Configs();

  //   cfg.kP = gains.kP();
  //   cfg.kS = gains.kS();
  //   cfg.kV = gains.kV();

  //   kickerMotor.getConfigurator().apply(cfg);
  // }

  @Override
  public void periodic() {
    log();

    switch (state) {
      case SHOOTING:
        if (RobotState.getInstance().isShooterReady) {
          setState(KickerState.IN);
        } else {
          setState(KickerState.STOP);
        }
        break;
      case STOP:
        kickerMotor.stopMotor();

      default:
        kickerMotor.set(state.getPower());
        break;
    }

    if (tuningMode.get()) {
      isTuning = true;
      setState(KickerState.IN);
    }

    if (!tuningMode.get() && isTuning) {
      setState(KickerState.STOP);
      isTuning = false;
    }
  }
}
