// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.HubStateUtil;
import java.lang.reflect.Field;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public static boolean autoSelected = false;

  private double loops = 0.0;

  public Robot() {
    SignalLogger.stop();

    StatusLogger.disableAutoLogging();
    StatusLogger.stop();

    // Adjust loop overrun warning timeout
    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(0.2);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }
    CommandScheduler.getInstance().setPeriod(0.2);

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    loops++;

    ShotCalculator.getInstance().clearParams();
    RobotContainer.getDeploySubsystem().clearCache();
    RobotContainer.getTurretSubsystem().clearCache();
    RobotContainer.getHoodSubsystem().clearCache();

    RobotContainer.getLeftVision().clearCache();
    RobotContainer.getRightVision().clearCache();

    RobotState.getInstance().periodic();
    CommandScheduler.getInstance().run();

    Mechanism3d.getInstance().log();
    HubStateUtil.log();

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putBoolean("Auto Selected", autoSelected);
    SmartDashboard.putData(CommandScheduler.getInstance());

    DogLog.log(
        "Loop/Avg Loop Time",
        Seconds.of(Timer.getFPGATimestamp() / loops).in(Milliseconds),
        Milliseconds);
    DogLog.log("Loop/Frequency", loops / Timer.getFPGATimestamp(), Hertz);
    DogLog.log("Loop/Total Loops", loops);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    m_robotContainer.getAutoSelector().updateChooser();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
