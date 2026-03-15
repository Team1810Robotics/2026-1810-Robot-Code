// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.state.RobotState;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.HubStateUtil;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public static boolean autoSelected = false;

  public Robot() {
    SignalLogger.stop();

    StatusLogger.disableAutoLogging();
    StatusLogger.stop();

    m_robotContainer = new RobotContainer();

    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    ShotCalculator.getInstance().clearParams();
    RobotContainer.getDeploySubsystem().clearCache();
    RobotContainer.getTurretSubsystem().clearCache();
    RobotContainer.getHoodSubsystem().clearCache();

    RobotContainer.getLeftVision().clearCache();
    RobotContainer.getRightVision().clearCache();

    CommandScheduler.getInstance().run();

    Mechanism3d.getInstance().log();
    HubStateUtil.log();
    RobotState.getInstance().log();

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putBoolean("Auto Selected", autoSelected);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    m_robotContainer.getAutoSelector().updateChooser();

    if (m_robotContainer.getAutonomousCommand() != Commands.none()
        || !m_robotContainer.getAutonomousCommand().getName().equals("No Auto")) {
      autoSelected = true;
    } else {
      autoSelected = false;
    }
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
