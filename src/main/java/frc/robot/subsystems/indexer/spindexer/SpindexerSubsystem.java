package frc.robot.subsystems.indexer.spindexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state.RobotState;
import frc.robot.subsystems.indexer.spindexer.SpindexerConstants.SpindexerState;
import frc.robot.util.TunablePIDF;
import frc.robot.util.TunablePIDF.TunablePIDFGains;

public class SpindexerSubsystem extends SubsystemBase {
  private final SparkMax spinMotor;
  private SparkClosedLoopController controller;

  private SpindexerState state = SpindexerState.STOP;

  private final BooleanSubscriber tuningMode = DogLog.tunable("Spindexer/Tuning Mode", false);
  private final DoubleSubscriber tuningTarget = DogLog.tunable("Spindexer/Tuning Target", 0.0);
  private boolean isTuning = false;

  private double velocityTarget = 0;

  private final TunablePIDF tunablePIDF;

  public SpindexerSubsystem() {
    spinMotor = new SparkMax(SpindexerConstants.SPIN_MOTOR, MotorType.kBrushless);

    SparkMaxConfig spinMotorConfig = new SparkMaxConfig();
    spinMotorConfig.idleMode(IdleMode.kCoast);
    spinMotorConfig.smartCurrentLimit(40);
    spinMotorConfig.inverted(true);

    ClosedLoopConfig clCfg = new ClosedLoopConfig();
    clCfg.p(SpindexerConstants.kP).feedForward.kV(SpindexerConstants.kV).kS(SpindexerConstants.kS);

    spinMotorConfig.apply(clCfg);

    spinMotor.configure(
        spinMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    controller = spinMotor.getClosedLoopController();

    state = SpindexerState.STOP;

    tunablePIDF = new TunablePIDF("Spindexer");
  }

  public void setState(SpindexerState state) {
    this.state = state;

    if (state == SpindexerState.STOP) {
      stop();
      return;
    }

    spinMotor.set(state.getVelocity());
  }

  public void setVelocity(double velocity) {
    velocityTarget = velocity;

    controller.setSetpoint(velocity, ControlType.kVelocity);
  }

  public Command spinCommand(SpindexerState state) {
    return Commands.startEnd(() -> setState(state), () -> stop(), this);
  }

  public boolean isJammed() {
    double target = state.getVelocity();
    double velocity = spinMotor.getEncoder().getVelocity();
    if (target > velocity) return false;

    return Math.abs(state.getVelocity() - spinMotor.getEncoder().getVelocity()) > 1000;
  }

  public void stop() {
    spinMotor.stopMotor();
  }

  public void log() {
    DogLog.log("Spindexer/State", state);
    DogLog.log("Spindexer/Velocity", spinMotor.getEncoder().getVelocity());
    DogLog.log("Spindexer/Is Jammed", isJammed());
  }

  public void updateGains() {
    TunablePIDFGains gains = tunablePIDF.getGains();
    if (!gains.hasChanged()) {
      return;
    }

    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.closedLoop.p(gains.kP()).feedForward.kS(gains.kS()).kV(gains.kV());

    spinMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    controller = spinMotor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    log();
    // updateGains();

    if (tuningMode.get()) {
      isTuning = true;
      controller.setSetpoint(tuningTarget.get(), ControlType.kVelocity);
    }

    if (!tuningMode.get() && isTuning) {
      setState(SpindexerState.STOP);
      isTuning = false;
    }

    switch (state) {
      case SHOOTING:
        if (RobotState.getInstance().isShooterReady) {
          setVelocity(SpindexerConstants.SpindexerState.IN.getVelocity());
        } else {
          stop();
        }
        break;

      case STOP:
        spinMotor.stopMotor();
        break;
      default:
        setVelocity(state.getVelocity());
        break;
    }
  }
}
