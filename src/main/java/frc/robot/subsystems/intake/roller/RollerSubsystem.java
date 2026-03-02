package frc.robot.subsystems.intake.roller;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.roller.RollerConstants.rollerState;

public class RollerSubsystem extends SubsystemBase {
  private final SparkMax rollerMotor;
  private final SparkClosedLoopController rollerController;

  private rollerState rollerState;

  public RollerSubsystem() {
    rollerMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    rollerMotorConfig.idleMode(IdleMode.kCoast);
    rollerMotorConfig.smartCurrentLimit(40);
    rollerMotorConfig.inverted(true);

    rollerMotorConfig.closedLoop.p(0).feedForward.kV(0).kS(0);

    rollerMotor.configure(
        rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rollerController = rollerMotor.getClosedLoopController();

    rollerState = RollerConstants.rollerState.STOP;
  }

  public void roller(rollerState state) {
    this.rollerState = state;
    if (state == RollerConstants.rollerState.STOP) {
      rollerMotor.stopMotor();
      return;
    }

    rollerController.setSetpoint(state.getVelocity(), ControlType.kVelocity);
  }

  public Command rollerCommand(rollerState state) {
    return Commands.run(() -> roller(state), this).finallyDo(() -> stopRoller());
  }

  public void stopRoller() {
    rollerMotor.stopMotor();
  }

  public void log() {
    DogLog.log("Intake/Roller/State", rollerState.name());
    DogLog.log("Intake/Roller/Target Velocity", rollerState.getVelocity());

    DogLog.log("Intake/Roller/Velocity", rollerMotor.getEncoder().getVelocity());
    DogLog.log("Intake/Roller/Voltage", rollerMotor.getBusVoltage());
  }

  @Override
  public void periodic() {
      log();
  }
}
