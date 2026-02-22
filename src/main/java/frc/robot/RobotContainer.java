// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;

@SuppressWarnings("unused")
public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * .25;

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
  private static final VisionSubsystem frontVision =
      new VisionSubsystem(VisionConstants.FRONT_LIMELIGHT_NAME);
  private static final VisionSubsystem rearVision =
      new VisionSubsystem(VisionConstants.REAR_LIMELIGHT_NAME);
  private static final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private static final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private static final FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
  private static final HoodSubsystem hoodSubsystem = new HoodSubsystem();

  public RobotContainer() {
    configureBindings();
    configureDogLog();
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-driverXbox.getLeftY() * MaxSpeed)
                    .withVelocityY(-driverXbox.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driverXbox.getRightX() * MaxAngularRate)));

    driverXbox.a().whileTrue(drivetrain.applyRequest(() -> brake));

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

    // reset the field-centric heading on left bumper press
    driverXbox.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    driverXbox.rightBumper().whileTrue(new Shoot());
  }

  /** Configure DogLog options and PowerDistribution */
  public void configureDogLog() {
    DogLog.setOptions(
        new DogLogOptions().withCaptureDs(true).withNtPublish(true).withLogExtras(true));

    DogLog.setPdh(new PowerDistribution());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public static CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
  }

  public static VisionSubsystem getFrontVisionSubsystem() {
    return frontVision;
  }

  public static VisionSubsystem getRearVisionSubsystem() {
    return rearVision;
  }

  public static IndexerSubsystem getIndexerSubsystem() {
    return indexerSubsystem;
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
}
