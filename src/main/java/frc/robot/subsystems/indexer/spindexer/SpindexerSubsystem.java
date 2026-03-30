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
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Timer;
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

  private SpindexerState spindexerState = SpindexerState.STOP;

  private final BooleanSubscriber tuningMode = DogLog.tunable("Spindexer/Tuning Mode", false);
  private final DoubleSubscriber tuningTarget = DogLog.tunable("Spindexer/Tuning Target", 0.0);
  private boolean isTuning = false;

  private double velocityTarget = 0;

  private final TunablePIDF tunablePIDF;

  private double startSpinUpTime;

  private boolean wasShooting = false;

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

    spindexerState = SpindexerState.STOP;

    tunablePIDF = new TunablePIDF("Spindexer");
  }

  public void setSpindexerState(SpindexerState state) {
    this.spindexerState = state;
  }

  public Command spinCommand(SpindexerState state) {
    return Commands.startEnd(() -> setSpindexerState(state), () -> stop(), this);
  }

  public boolean isJammed() {
    if (Timer.getFPGATimestamp() - startSpinUpTime < SpindexerConstants.SPINUP_TIME) return false;
    if (spindexerState == SpindexerState.STOP || spindexerState == SpindexerState.OUT) return false;
    if (!RobotState.getInstance().isShooterReady) return false;

    return spinMotor.getOutputCurrent() > 38.0;
  }

  public void stop() {
    spinMotor.stopMotor();
  }

  public void log() {
    DogLog.log("Spindexer/State", spindexerState);
    DogLog.log("Spindexer/Is Jammed", isJammed());
    DogLog.log("Spindexer/Current", spinMotor.getOutputCurrent());
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

    // if (tuningMode.get()) {
    //   isTuning = true;
    //   spinMotor.set(SpindexerConstants.SpindexerState.IN.getPower());
    // }

    // if (!tuningMode.get() && isTuning) {
    //   spinMotor.stopMotor();
    //   // setSpindexerState(SpindexerState.STOP);
    //   isTuning = false;
    // }

    switch (spindexerState) {
      case SHOOTING:
        if (RobotState.getInstance().isShooterReady) {
          if (!wasShooting) {
            startSpinUpTime = Timer.getFPGATimestamp();
            wasShooting = true;
          }
          spinMotor.set(spindexerState.getPower());
          // setVelocity(SpindexerConstants.SpindexerState.IN.getVelocity());
        } else {
          stop();
          wasShooting = false;
        }
        break;

      case STOP:
        spinMotor.stopMotor();
        break;
      default:
        spinMotor.set(spindexerState.getPower());
        break;
    }
  }
}
