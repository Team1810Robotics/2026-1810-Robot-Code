package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.intakeState;

public class IndexerSubsystem extends SubsystemBase {
  private SparkMax spinMotor;
  private SparkMax kickerMotor;

  public IndexerSubsystem() {
    spinMotor = new SparkMax(IndexerConstants.SPIN_MOTOR, MotorType.kBrushless);
    kickerMotor = new SparkMax(IndexerConstants.KICKER_MOTOR, MotorType.kBrushless);

    SparkMaxConfig spinMotorConfig = new SparkMaxConfig();
    spinMotorConfig.idleMode(IdleMode.kCoast);
    spinMotorConfig.smartCurrentLimit(40);

    spinMotor.configure(
        spinMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig kickerMotorConfig = new SparkMaxConfig();
    kickerMotorConfig.idleMode(IdleMode.kCoast);
    kickerMotorConfig.smartCurrentLimit(40);

    kickerMotor.configure(
        kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intake(intakeState state) {
    if (state == intakeState.STOP) {
      fullstop();
      return;
    }

    spinMotor.set(state.getPower());
    kickerMotor.set(state.getPower());
  }

  public void run(double spinSpeed, double kickSpeed) {
    spinMotor.set(spinSpeed);
    kickerMotor.set(kickSpeed);
  }

  public void fullstop() {
    spinMotor.stopMotor();
    kickerMotor.stopMotor();
  }

  public void spinstop() {
    spinMotor.stopMotor();
  }

  public void feedstop() {
    kickerMotor.stopMotor();
  }
}
