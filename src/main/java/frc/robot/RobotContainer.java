// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState.RobotStates;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.indexer.kicker.KickerConstants.KickerState;
import frc.robot.subsystems.indexer.kicker.KickerSubsystem;
import frc.robot.subsystems.indexer.spindexer.SpindexerConstants.SpindexerState;
import frc.robot.subsystems.indexer.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.intake.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;

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

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();
    configureDogLog();

    autoChooser = AutoBuilder.buildAutoChooser("No Auto");
    SmartDashboard.putData(autoChooser);
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-driverXbox.getLeftY() * MaxSpeed)
                    .withVelocityY(-driverXbox.getLeftX() * MaxSpeed)
                    .withRotationalRate(Robot.isSimulation() ? -driverXbox.getRawAxis(2) : -driverXbox.getRightX() * MaxAngularRate)));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driverXbox.back().and(driverXbox.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverXbox.back().and(driverXbox.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverXbox
        .start()
        .and(driverXbox.y())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverXbox
        .start()
        .and(driverXbox.x())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    driverXbox
        .a()
        .and(RobotState.getInstance().checkRobotState(RobotStates.NEUTRAL))
        .onTrue(RobotState.getInstance().advanceIntakeState());

    driverXbox
        .rightBumper()
        .whileTrue(
            Commands.sequence(
                RobotState.getInstance().setStateCommand(RobotStates.SHOOTING), new Shoot()));

    driverXbox
        .x()
        .whileTrue(
            Commands.parallel(
                kickerSubsystem.kickCommand(KickerState.OUT),
                spindexerSubsystem.spinCommand(SpindexerState.OUT)));

    driverXbox.povDown().onTrue(hoodSubsystem.zero());

    // reset the field-centric heading on left bumper press
    driverXbox.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
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
  }

  /** Configure DogLog options and PowerDistribution */
  public void configureDogLog() {
    // DogLog.setPdh(new PowerDistribution(63, ModuleType.kRev));

    DogLog.setOptions(
        new DogLogOptions().withCaptureDs(true).withNtPublish(true).withLogExtras(true));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
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
