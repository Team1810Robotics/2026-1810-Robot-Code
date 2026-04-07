package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.ShotParameters;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelState;

public class FlywheelSubsystem extends SubsystemBase {
  private final TalonFX rightMotor;
  private final TalonFX leftMotor;

  // private final TunablePIDF frontTunablePIDF;
  // private final TunablePIDF backTunablePIDF;

  private final DoubleSubscriber velocityTarget = DogLog.tunable("Flywheel/VelocityTarget", 0.0);

  private AngularVelocity frontTargetVelocity = RotationsPerSecond.of(0);
  private AngularVelocity backTargetVelocity = RotationsPerSecond.of(0);

  private FlywheelState flywheelState = FlywheelState.IDLE;

  public FlywheelSubsystem() {
    rightMotor = new TalonFX(FlywheelConstants.RIGHT_FLYWHEEL_ID);
    leftMotor = new TalonFX(FlywheelConstants.LEFT_FLYWHEEL_ID);

    TalonFXConfiguration frontConfig = new TalonFXConfiguration();

    frontConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    frontConfig.CurrentLimits.StatorCurrentLimit = 120;

    frontConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    frontConfig.CurrentLimits.SupplyCurrentLimit = 40;

    frontConfig.MotorOutput.PeakForwardDutyCycle = 1;
    frontConfig.MotorOutput.PeakReverseDutyCycle = 0;

    frontConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    frontConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    frontConfig.Slot0.kP = FlywheelConstants.frontkP;
    frontConfig.Slot0.kI = FlywheelConstants.frontkI;
    frontConfig.Slot0.kD = FlywheelConstants.frontkD;

    frontConfig.Slot0.kS = FlywheelConstants.frontkS;
    frontConfig.Slot0.kV = FlywheelConstants.frontkV;

    rightMotor.getConfigurator().apply(frontConfig);

    TalonFXConfiguration backConfig = frontConfig;

    backConfig.Slot0.kP = FlywheelConstants.backkP;
    backConfig.Slot0.kI = FlywheelConstants.backkI;
    backConfig.Slot0.kD = FlywheelConstants.backkD;
    backConfig.Slot0.kS = FlywheelConstants.backkS;
    backConfig.Slot0.kV = FlywheelConstants.backkV;

    leftMotor.getConfigurator().apply(backConfig);

    // frontTunablePIDF = new TunablePIDF("Flywheel/Front");
    // backTunablePIDF = new TunablePIDF("Flywheel/Back");
  }

  public Command dutyCycleCommand(double dutyCycle) {
    return Commands.run(() -> rightMotor.setControl(new DutyCycleOut(dutyCycle)), this)
        .finallyDo(() -> stop());
  }

  public void idleMotor() {
    rightMotor.setControl(new DutyCycleOut(.2));
  }

  public Command idleMotorCommand() {
    return Commands.startEnd(
        () -> {
          idleMotor();
        },
        () -> stop(),
        this);
  }

  public void setState(FlywheelState state) {
    this.flywheelState = state;
  }

  public void setVelocity(AngularVelocity velocity) {
    frontTargetVelocity = velocity;
    backTargetVelocity = velocity.times(.95); // TODO: Add the times back

    rightMotor.setControl(new VelocityVoltage(frontTargetVelocity));
    leftMotor.setControl(new VelocityVoltage(backTargetVelocity));
  }

  public Command setVelocityCommand(AngularVelocity velocity) {
    return Commands.runOnce(() -> setVelocity(velocity), this).finallyDo(() -> stop());
  }

  public boolean atTargetVelocity() {
    if (rightMotor.getVelocity().getValueAsDouble() > frontTargetVelocity.in(RotationsPerSecond))
      return true;
    return Math.abs(
            rightMotor.getVelocity().getValueAsDouble()
                - frontTargetVelocity.in(RotationsPerSecond))
        < 2; // TODO: Tune this threshold
  }

  public void stop() {
    rightMotor.stopMotor();
  }

  // public void updateGains() {
  //   TunablePIDFGains frontGains = frontTunablePIDF.getGains();
  //   TunablePIDFGains backGains = backTunablePIDF.getGains();

  //   if (!frontGains.hasChanged() && !backGains.hasChanged()) {
  //     return;
  //   }

  //   Slot0Configs frontGainConfig = new Slot0Configs();
  //   frontGainConfig.kP = frontGains.kP();
  //   frontGainConfig.kI = frontGains.kI();
  //   frontGainConfig.kD = frontGains.kD();
  //   frontGainConfig.kS = frontGains.kS();
  //   frontGainConfig.kV = frontGains.kV();

  //   rightMotor.getConfigurator().apply(frontGainConfig);

  //   Slot0Configs backGainConfig = new Slot0Configs();
  //   backGainConfig.kP = frontGains.kP();
  //   backGainConfig.kI = frontGains.kI();
  //   backGainConfig.kD = frontGains.kD();
  //   backGainConfig.kS = frontGains.kS();
  //   backGainConfig.kV = frontGains.kV();

  //   leftMotor.getConfigurator().apply(backGainConfig);
  // }

  public void log() {
    DogLog.log(
        "Flywheel/Front/Velocity", rightMotor.getVelocity().getValueAsDouble(), RotationsPerSecond);
    DogLog.log(
        "Flywheel/Front/Target", frontTargetVelocity.in(RotationsPerSecond), RotationsPerSecond);
    DogLog.log("Flywheel/Front/Voltage", rightMotor.getMotorVoltage().getValueAsDouble(), Volts);

    DogLog.log(
        "Flywheel/Back/Velocity", leftMotor.getVelocity().getValueAsDouble(), RotationsPerSecond);
    DogLog.log(
        "Flywheel/Back/Target", backTargetVelocity.in(RotationsPerSecond), RotationsPerSecond);
    DogLog.log(
        "Flywheel/Back/Voltage",
        leftMotor.getMotorVoltage().getValueAsDouble(),
        RotationsPerSecond);
  }

  @Override
  public void periodic() {
    log();
    // updateGains();

    // if (velocityTarget.get() != 0) {
    //   setVelocity(RotationsPerSecond.of(velocityTarget.get()));
    // } else {
    //   stop();
    // }

    switch (flywheelState) {
      case PASSING:
        ShotParameters passingParams = ShotCalculator.getInstance().calculateParameters();

        if (!passingParams.isValid()) {
          idleMotor();
        } else {
          setVelocity(passingParams.flywheelVelocity());
        }
        break;
      case SCORING:
        ShotParameters scoringParams = ShotCalculator.getInstance().calculateParameters();

        if (!scoringParams.isValid()) {
          idleMotor();
        } else {
          setVelocity(scoringParams.flywheelVelocity());
        }
        break;
      case IDLE:
        idleMotor();

        break;
      default:
        break;
    }
  }
}
