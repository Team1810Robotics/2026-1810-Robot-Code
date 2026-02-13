package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {
  private final TalonFX rightMotor;
  private final TalonFX leftMotor;

  private final DoubleSubscriber kP = DogLog.tunable("Flywheel/Gains/kP", 0.0);
  private final DoubleSubscriber kI = DogLog.tunable("Flywheel/Gains/kI", 0.0);
  private final DoubleSubscriber kD = DogLog.tunable("Flywheel/Gains/kD", 0.0);
  private final DoubleSubscriber kS = DogLog.tunable("Flywheel/Gains/kS", 0.0);
  private final DoubleSubscriber kV = DogLog.tunable("Flywheel/Gains/kV", 0.0);
  private final DoubleSubscriber kA = DogLog.tunable("Flywheel/Gains/kA", 0.0);

  private double targetVelocity = 0;

  public FlywheelSubsystem() {
    rightMotor = new TalonFX(FlywheelConstants.RIGHT_FLYWHEEL_ID);
    leftMotor = new TalonFX(FlywheelConstants.LEFT_FLYWHEEL_ID);

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 120;

    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 40;

    cfg.MotorOutput.PeakForwardDutyCycle = 1;
    cfg.MotorOutput.PeakReverseDutyCycle = 0;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    cfg.Slot0.kP = FlywheelConstants.kP;
    cfg.Slot0.kI = FlywheelConstants.kI;
    cfg.Slot0.kD = FlywheelConstants.kD;

    cfg.Slot0.kS = FlywheelConstants.kS;
    cfg.Slot0.kV = FlywheelConstants.kV;
    cfg.Slot0.kA = FlywheelConstants.kA;

    rightMotor.getConfigurator().apply(cfg);
    leftMotor.getConfigurator().apply(cfg);

    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), MotorAlignmentValue.Aligned));

    setDefaultCommand(idleMotorCommand());
  }

  public void idleMotor() {
    rightMotor.setControl(new DutyCycleOut(.2));
  }

  public Command idleMotorCommand() {
    return Commands.startEnd(() -> idleMotor(), () -> stop(), this);
  }

  public void setVelocity(AngularVelocity velocity) {
    targetVelocity = velocity.in(RadiansPerSecond);
    rightMotor.setControl(new VelocityVoltage(velocity));
  }

  public Command setVelocityCommand(AngularVelocity velocity) {
    return Commands.startEnd(() -> setVelocity(velocity), () -> stop(), this);
  }

  public boolean atTargetVelocity() {
    return Math.abs(rightMotor.getVelocity().getValueAsDouble() - targetVelocity)
        < 10; // TODO: Tune this threshold
  }

  public void stop() {
    rightMotor.stopMotor();
  }

  public void updateGains() {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kA = kA.get();

    rightMotor.getConfigurator().apply(slot0Configs);
  }

  public void log() {
    DogLog.log(
        "Flywheel/Velocity",
        RotationsPerSecond.of(rightMotor.getVelocity().getValueAsDouble()).in(RadiansPerSecond),
        RadiansPerSecond);
    DogLog.log("Flywheel/Target", targetVelocity, RadiansPerSecond);
    DogLog.log("Flywheel/Connected", rightMotor.isConnected());
    DogLog.log(
        "Flywheel/Acceleration",
        RotationsPerSecondPerSecond.of(rightMotor.getAcceleration().getValueAsDouble())
            .in(RadiansPerSecondPerSecond),
        RadiansPerSecondPerSecond);
    DogLog.log("Flywheel/Voltage", rightMotor.getMotorVoltage().getValueAsDouble(), Volts);
  }

  @Override
  public void periodic() {
    log();
  }
}
