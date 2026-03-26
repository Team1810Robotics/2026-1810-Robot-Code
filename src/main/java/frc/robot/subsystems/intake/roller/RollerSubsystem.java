package frc.robot.subsystems.intake.roller;

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
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state.RobotState;
import frc.robot.subsystems.intake.roller.RollerConstants.RollerState;

public class RollerSubsystem extends SubsystemBase {
  private final SparkMax rollerMotor;
  private SparkClosedLoopController rollerController;

  private RollerState rollerState = RollerState.STOP;

  private DoubleSubscriber kP = DogLog.tunable("Intake/Roller/kP", 0.0);
  private DoubleSubscriber kS = DogLog.tunable("Intake/Roller/kS", 0.1);
  private DoubleSubscriber kV = DogLog.tunable("Intake/Roller/kV", 0.0025);

  private double lastkP, lastkS, lastkV;

  private DoubleSubscriber target = DogLog.tunable("Intake/Roller/Target", 0.0);

  public RollerSubsystem() {
    rollerMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    rollerMotorConfig.idleMode(IdleMode.kCoast);
    rollerMotorConfig.smartCurrentLimit(40);
    rollerMotorConfig.inverted(false);

    rollerMotorConfig
        .closedLoop
        .p(RollerConstants.kP)
        .feedForward
        .kV(RollerConstants.kV)
        .kS(RollerConstants.kS);

    rollerMotor.configure(
        rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rollerController = rollerMotor.getClosedLoopController();

    rollerState = RollerConstants.RollerState.STOP;
  }

  public void setState(RollerState state) {
    this.rollerState = state;
  }

  public Command rollerCommand(RollerState state) {
    return Commands.run(() -> setState(state), this).finallyDo(() -> stop());
  }

  public void stop() {
    rollerMotor.stopMotor();
  }

  public void log() {
    DogLog.log("Intake/Roller/Roller State", rollerState.name());
    DogLog.log("Intake/Roller/Target Velocity", rollerState.getVelocity());

    DogLog.log("Intake/Roller/Velocity", rollerMotor.getEncoder().getVelocity());
    DogLog.log("Intake/Roller/Voltage", rollerMotor.getBusVoltage());
  }

  public void updateGains() {
    if (kP.get() != lastkP || kS.get() != lastkS || kV.get() != lastkV) {
      SparkMaxConfig spmCFG = new SparkMaxConfig();

      ClosedLoopConfig cfg = new ClosedLoopConfig();
      cfg.p(kP.get()).feedForward.kV(kV.get()).kS(kS.get());

      spmCFG.apply(cfg);

      rollerMotor.configure(spmCFG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      rollerController = rollerMotor.getClosedLoopController();
    }
  }

  @Override
  public void periodic() {
    log();

    switch (rollerState) {
      case STOP:
        rollerMotor.stopMotor();
        break;
      case SHOOTING:
        if (RobotState.getInstance().isShooterReady) {
          rollerController.setSetpoint(RollerState.INTAKE.getVelocity(), ControlType.kVelocity);
        } else {
          rollerMotor.stopMotor();
        }
        break;
      default:
        rollerController.setSetpoint(rollerState.getVelocity(), ControlType.kVelocity);
        break;
    }
  }
}
