package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.flywheel.FlywheelConstants;

public class HoodSubsystem extends SubsystemBase {
  private final TalonFX hoodMotor;

  private final DoubleSubscriber kP = DogLog.tunable("Hood/Gains/kP", 0.0);
  private final DoubleSubscriber kI = DogLog.tunable("Hood/Gains/kI", 0.0);
  private final DoubleSubscriber kD = DogLog.tunable("Hood/Gains/kD", 0.0);
  private final DoubleSubscriber kS = DogLog.tunable("Hood/Gains/kS", 0.0);
  private final DoubleSubscriber kV = DogLog.tunable("Hood/Gains/kV", 0.0);
  private final DoubleSubscriber kA = DogLog.tunable("Hood/Gains/kA", 0.0);

  public HoodSubsystem() {
    hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID);

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 120;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 40;

    cfg.Feedback.SensorToMechanismRatio = 0; // TODO: go harass austin

    cfg.Slot0.kP = FlywheelConstants.kP;
    cfg.Slot0.kI = FlywheelConstants.kI;
    cfg.Slot0.kD = FlywheelConstants.kD;

    cfg.Slot0.kS = FlywheelConstants.kS;
    cfg.Slot0.kV = FlywheelConstants.kV;
    cfg.Slot0.kA = FlywheelConstants.kA;

    hoodMotor.getConfigurator().apply(cfg);

    hoodMotor.setPosition(Rotations.of(0));
  }

  public void setPosition(Rotation2d position) {
    hoodMotor.setControl(new PositionVoltage(position.getRotations()));
  }

  public Command setPositionCommand(Rotation2d position) {
    return Commands.startEnd(() -> setPosition(position), () -> stop(), this);
  }

  public Rotation2d getPosition() {
    return Rotation2d.fromRotations(hoodMotor.getPosition().getValueAsDouble());
  }

  public void updateGains() {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kA = kA.get();

    hoodMotor.getConfigurator().apply(slot0Configs);
  }

  public void stop() {
    hoodMotor.stopMotor();
  }
}
