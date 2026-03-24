// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.AutoSelector;
import frc.robot.state.RobotState;
import frc.robot.state.RobotState.RobotStates;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.indexer.kicker.KickerSubsystem;
import frc.robot.subsystems.indexer.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.intake.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.field.Region;

@SuppressWarnings("unused")
public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * .5;

  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private static final CommandXboxController driverXbox =
      new CommandXboxController(0); // controllers
  private final CommandXboxController operatorXbox = new CommandXboxController(1);

  // subsystems :)
  private static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Vision objects
  private static final Vision leftVision = new Vision(VisionConstants.LEFT_LIMELIGHT_NAME);
  private static final Vision rightVision = new Vision(VisionConstants.RIGHT_LIMELIGHT_NAME);

  // Shooter Subsystems
  private static final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private static final FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
  private static final HoodSubsystem hoodSubsystem = new HoodSubsystem();

  // private static final LEDSubsystem ledSubsystem = new LEDSubsystem();

  // Intake Subystems
  private static final DeploySubsystem deploySubsystem = new DeploySubsystem();
  private static final RollerSubsystem rollerSubsystem = new RollerSubsystem();

  // Indexer Subsystems
  private static final SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem();
  private static final KickerSubsystem kickerSubsystem = new KickerSubsystem();

  public boolean manualTurret = false;
  public boolean killManual = true;

  private final AutoSelector autoSelector = new AutoSelector();

  public RobotContainer() {
    configureBindings();
    configureDogLog();
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-driverXbox.getLeftY() * MaxSpeed * .5)
                    .withVelocityY(-driverXbox.getLeftX() * MaxSpeed * .5)
                    .withRotationalRate(-driverXbox.getRightX() * MaxAngularRate * .5)));

    driverXbox
        .leftBumper()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(-driverXbox.getLeftY() * MaxSpeed)
                        .withVelocityY(-driverXbox.getLeftX() * MaxSpeed)
                        .withRotationalRate(-driverXbox.getRightX() * MaxAngularRate)));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // driverXbox.back().and(driverXbox.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // driverXbox.back().and(driverXbox.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // driverXbox
    //     .start()
    //     .and(driverXbox.y())
    //     .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // driverXbox
    //     .start()
    //     .and(driverXbox.x())
    //     .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    driverXbox
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState robotState = RobotState.getInstance();
                  switch (RobotState.getInstance().getState()) {
                    case NEUTRAL:
                      robotState.setState(RobotStates.INTAKING);
                      break;
                    case INTAKING:
                      robotState.setState(RobotStates.NEUTRAL);
                      break;
                    default:
                      break;
                  }
                }));

    driverXbox
        .rightTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  Region botRegion = Region.getRegion(drivetrain.getPose()).orElseThrow();

                  if (botRegion == Region.BLUE_ALLIANCE_ZONE
                      || botRegion == Region.RED_ALLIANCE_ZONE) {
                    RobotState.getInstance().setState(RobotStates.SCORING_NO_AGITATION);
                  } else {
                    RobotState.getInstance().setState(RobotStates.PASSING);
                  }
                }));

    driverXbox
        .leftTrigger()
        .onTrue(RobotState.getInstance().setStateCommand(RobotStates.SCORING_WITH_AGITATION));

    driverXbox.a().onTrue(RobotState.getInstance().setStateCommand(RobotStates.REVERSE_INDEXER));

    driverXbox.povDown().onTrue(hoodSubsystem.zero());

    // reset the field-centric heading on left bumper press
    driverXbox.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    driverXbox
        .leftStick()
        .onTrue(
            Commands.runOnce(
                () ->
                    drivetrain.resetPose(
                        new Pose2d(
                            Inches.of(27 / 2).in(Meters),
                            Inches.of(27 / 2).in(Meters),
                            Rotation2d.kZero))));

    operatorXbox
        .rightStick()
        .onTrue(
            Commands.run(
                    () -> {
                      turretSubsystem.set(operatorXbox.getRightX() / 10);
                    },
                    turretSubsystem)
                .until(() -> operatorXbox.b().getAsBoolean())
                .andThen(Commands.runOnce(() -> turretSubsystem.seedMotorFromAbsolute())));
  }

  /** Configure DogLog options and PowerDistribution */
  public void configureDogLog() {
    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withNtPublish(true)
            .withLogExtras(false)
            .withCaptureNt(false)
            .withCaptureConsole(false));
  }

  public Command getAutonomousCommand() {
    if (autoSelector.getSelectedAuto() != null) {
      return autoSelector.getSelectedAuto();
    }

    return Commands.none();
  }

  public AutoSelector getAutoSelector() {
    return autoSelector;
  }

  public static CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
  }

  public static Vision getLeftVision() {
    return leftVision;
  }

  public static Vision getRightVision() {
    return rightVision;
  }

  public static TurretSubsystem getTurretSubsystem() {
    return turretSubsystem;
  }

  public static FlywheelSubsystem getFlywheelSubsystem() {
    return flywheelSubsystem;
  }

  public static HoodSubsystem getHoodSubsystem() {
    return hoodSubsystem;
  }

  public static DeploySubsystem getDeploySubsystem() {
    return deploySubsystem;
  }

  public static RollerSubsystem getRollerSubsystem() {
    return rollerSubsystem;
  }

  public static SpindexerSubsystem getSpindexerSubsystem() {
    return spindexerSubsystem;
  }

  public static KickerSubsystem getKickerSubsystem() {
    return kickerSubsystem;
  }
}
