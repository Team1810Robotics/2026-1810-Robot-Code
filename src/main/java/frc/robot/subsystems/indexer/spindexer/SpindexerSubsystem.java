package frc.robot.subsystems.indexer.spindexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.spindexer.SpindexerConstants.SpindexerState;

public class SpindexerSubsystem extends SubsystemBase {
  private final SparkMax spinMotor;
  private SparkClosedLoopController controller;

  private SpindexerState state = SpindexerState.STOP;

  private final BooleanSubscriber tuningMode = DogLog.tunable("Spindexer/Tuning Mode", false);
  private boolean isTuning = false;

  // private final TunablePIDF tunablePIDF;

  public SpindexerSubsystem() {
    spinMotor = new SparkMax(SpindexerConstants.SPIN_MOTOR, MotorType.kBrushless);

    SparkMaxConfig spinMotorConfig = new SparkMaxConfig();
    spinMotorConfig.idleMode(IdleMode.kCoast);
    spinMotorConfig.smartCurrentLimit(40);
    spinMotorConfig.inverted(true);

    ClosedLoopConfig clCfg = new ClosedLoopConfig();
    clCfg.p(0).feedForward.kV(0).kS(0);

    spinMotorConfig.apply(clCfg);

    spinMotor.configure(
        spinMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    controller = spinMotor.getClosedLoopController();

    state = SpindexerState.STOP;

    // tunablePIDF = new TunablePIDF("Spindexer");
  }

  public void setState(SpindexerState state) {
    this.state = state;

    if (state == SpindexerState.STOP) {
      stop();
      return;
    }

    spinMotor.set(state.getPower());
  }

  public Command spinCommand(SpindexerState state) {
    return Commands.startEnd(() -> setState(state), () -> stop(), this);
  }

  public void stop() {
    spinMotor.stopMotor();
  }

  public void log() {
    DogLog.log("Spindexer/State", state);
  }

  // public void updateGains() {
  //   TunablePIDFGains gains = tunablePIDF.getGains();
  //   if (!gains.hasChanged()) {
  //     return;
  //   }

  //   SparkMaxConfig cfg = new SparkMaxConfig();
  //   cfg.closedLoop.p(gains.kP()).feedForward.kS(gains.kS()).kV(gains.kV());

  //   spinMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  //   controller = spinMotor.getClosedLoopController();
  // }

  @Override
  public void periodic() {
    log();
    // updateGains();

    if (tuningMode.get()) {
      isTuning = true;
      setState(SpindexerState.IN);
    }

    if (!tuningMode.get() && isTuning) {
      setState(SpindexerState.STOP);
      isTuning = false;
    }
  }
}
