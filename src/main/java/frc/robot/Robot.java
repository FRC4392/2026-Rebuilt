// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private final DeceiverRobotState robotState = new DeceiverRobotState();
  private final RobotContainer robotContainer = new RobotContainer(robotState);
  private Command autonomousCommand;

  public Robot() {
    // Record metadata about the git version for future reference
    Logger.recordMetadata("RobotMode", RobotConstants.currentMode.toString());
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up logging based on robot mode
    switch (RobotConstants.currentMode) {
      case COMMISIONING:
        // Don't log to network tables during real match
        // Probably always false at this point, but doesn't hurt to check
        if (!DriverStation.isFMSAttached()) {
          Logger.addDataReceiver(new NT4Publisher());
        }
        // Fall through
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start CTRE Logger
    SignalLogger.start();

    // Add Rev Logger

    // Start AdvantageKit Logger
    Logger.start();
  }

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void driverStationConnected() {}

  @Override
  public void disabledInit() {
    robotState.setDisabled(true);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    robotState.setDisabled(false);
  }

  @Override
  public void autonomousInit() {
    robotState.setAuto(true);

    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    robotState.setAuto(false);
  }

  @Override
  public void teleopInit() {
    robotState.setTeleop(true);

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    robotState.setTeleop(false);
  }

  @Override
  public void testInit() {
    robotState.setTest(true);

    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    robotState.setTest(false);
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
