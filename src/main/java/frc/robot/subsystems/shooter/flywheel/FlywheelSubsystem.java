package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

  private final DoubleSubscriber velocityTarget = DogLog.tunable("Flywheel/VelocityTarget", 0.0);

  private boolean isTuning = false;

  private double lastkP, lastkI, lastkD, lastkS, lastkV;

  private double targetVelocity = 0;

  private Command tuningCommand = setVelocityCommand(RotationsPerSecond.of(velocityTarget.get()));

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

    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    cfg.Slot0.kP = FlywheelConstants.kP;
    cfg.Slot0.kI = FlywheelConstants.kI;
    cfg.Slot0.kD = FlywheelConstants.kD;

    cfg.Slot0.kS = FlywheelConstants.kS;
    cfg.Slot0.kV = FlywheelConstants.kV;

    rightMotor.getConfigurator().apply(cfg);
    leftMotor.getConfigurator().apply(cfg);

    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), MotorAlignmentValue.Aligned));

    setDefaultCommand(idleMotorCommand());
  }

  public Command dutyCycleCommand(double dutyCycle) {
    return Commands.run(() -> rightMotor.setControl(new DutyCycleOut(dutyCycle)), this)
        .finallyDo(() -> stop());
  }

  public void idleMotor() {
    rightMotor.setControl(new DutyCycleOut(.2));
  }

  public Command idleMotorCommand() {
    return Commands.startEnd(() -> idleMotor(), () -> stop(), this);
  }

  public void setVelocity(AngularVelocity velocity) {
    targetVelocity = velocity.in(RotationsPerSecond);
    rightMotor.setControl(new VelocityVoltage(velocity));
  }

  public Command setVelocityCommand(AngularVelocity velocity) {
    return Commands.run(() -> setVelocity(velocity), this).finallyDo(() -> stop());
  }

  public boolean atTargetVelocity() {
    return Math.abs(rightMotor.getVelocity().getValueAsDouble() - targetVelocity)
        < 2; // TODO: Tune this threshold
  }

  public void stop() {
    rightMotor.stopMotor();
  }

  public void updateGains() {
    if (kP.get() == lastkP
        && kI.get() == lastkI
        && kD.get() == lastkD
        && kS.get() == lastkS
        && kV.get() == lastkV) {
      return;
    }

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();

    lastkP = kP.get();
    lastkI = kI.get();
    lastkD = kD.get();
    lastkS = kS.get();
    lastkV = kV.get();

    rightMotor.getConfigurator().apply(slot0Configs);
  }

  public void log() {
    DogLog.log(
        "Flywheel/Velocity", rightMotor.getVelocity().getValueAsDouble(), RotationsPerSecond);
    DogLog.log("Flywheel/Target", targetVelocity, RotationsPerSecond);
    DogLog.log("Flywheel/Connected", rightMotor.isConnected());
    DogLog.log(
        "Flywheel/Acceleration",
        rightMotor.getAcceleration().getValueAsDouble(),
        RotationsPerSecondPerSecond);
    DogLog.log("Flywheel/Voltage", rightMotor.getMotorVoltage().getValueAsDouble(), Volts);
    DogLog.log("Flywheel/At Target Velocity", atTargetVelocity());
  }

  @Override
  public void periodic() {
    log();

    if (velocityTarget.get() != 0) {
      isTuning = true;
      CommandScheduler.getInstance().schedule(tuningCommand);
    }

    if (isTuning && velocityTarget.get() == 0) {
      isTuning = false;
      CommandScheduler.getInstance().cancel(tuningCommand);
    }
  }
}
