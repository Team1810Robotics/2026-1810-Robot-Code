package frc.robot.subsystems.indexer.kicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.indexer.kicker.KickerConstants.KickerState;

public class KickerSubsystem extends SubsystemBase {
  private final TalonFX kickerMotor;

  // private final TunablePIDF tunablePIDF = new TunablePIDF("kicker");

  private final BooleanSubscriber tuningMode = DogLog.tunable("Kicker/Tuning Mode", false);
  private boolean isTuning = false;

  private KickerState state = KickerState.STOP;

  private double startSpinUpTime = 0;
  private boolean wasShooting = false;

  public KickerSubsystem() {
    this.kickerMotor = new TalonFX(KickerConstants.KICKER_MOTOR);

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 120;

    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 40;

    cfg.Feedback.SensorToMechanismRatio = 3;

    kickerMotor.getConfigurator().apply(cfg);
  }

  public void setState(KickerState state) {
    this.state = state;
  }

  public Command kickCommand(KickerState state) {
    return Commands.startEnd(() -> setState(state), () -> stop(), this);
  }

  public boolean isJammed() {
    double currentTime = Timer.getFPGATimestamp();

    // Give time to spin up
    if (currentTime - startSpinUpTime < KickerConstants.SPINUP_TIME) return false;

    // Ignore states where we shouldn't be moving forward
    if (state == KickerState.STOP || state == KickerState.OUT) return false;

    // Only care when shooter is ready (same as spindexer)
    if (!RobotState.getInstance().isShooterReady) return false;

    return kickerMotor.getStatorCurrent().getValueAsDouble() > KickerConstants.JAM_CURRENT.in(Amps);
  }

  public void stop() {
    kickerMotor.stopMotor();
  }

  public void log() {
    DogLog.log("Kicker/State", state);
    DogLog.log("Kicker/Velocity", kickerMotor.getVelocity().getValueAsDouble(), RotationsPerSecond);
    DogLog.log("Kicker/Is Jammed", isJammed());
    DogLog.log("Kicker/Current", kickerMotor.getStatorCurrent().getValueAsDouble(), Amps);
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

    // if (tuningMode.get()) {
    //   kickerMotor.set(KickerState.IN.getPower());
    // } else {
    //   stop();
    // }

    switch (state) {
      case SHOOTING:
        if (RobotState.getInstance().isShooterReady) {
          if (!wasShooting) {
            startSpinUpTime = Timer.getFPGATimestamp();
            wasShooting = true;
          }
          kickerMotor.set(KickerState.IN.getPower());
        } else {
          kickerMotor.stopMotor();
          wasShooting = false;
        }
        break;
      case STOP:
        kickerMotor.stopMotor();
        break;
      default:
        kickerMotor.set(state.getPower());
        break;
    }
  }
}
