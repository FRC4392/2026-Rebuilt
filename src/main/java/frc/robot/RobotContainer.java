// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.operatorinterface.OperatorInterface;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIONavx3;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIODeceivers;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;

public class RobotContainer {
  private final DeceiverRobotState robotState;

  // Subsystems
  public final Swerve swerve;

  // Operator Interface
  private final OperatorInterface operatorInterface;

  /**
   * Constructor
   *
   * @param state RobotState object to track the state of the robot
   */
  public RobotContainer(DeceiverRobotState state) {
    robotState = state;

    // Lower brownout voltage
    RobotController.setBrownoutVoltage(6.0);

    // Create subsystem hardware
    switch (RobotConstants.currentMode) {
      case COMMISIONING:
        // Fall Through
      case REAL:
        // Real Robot, use real hardware interfaces
        swerve =
            new Swerve(
                new GyroIONavx3(),
                new SwerveModuleIODeceivers(0),
                new SwerveModuleIODeceivers(1),
                new SwerveModuleIODeceivers(2),
                new SwerveModuleIODeceivers(3),
                state);
        break;
      case SIM:
        // Simulated robot use simulation hardware interfaces
        swerve =
            new Swerve(
                new GyroIO() {},
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                state);
        break;
      default:
        // Replay, don't use hardware
        swerve =
            new Swerve(
                new GyroIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                state);
    }

    // Create Operator Interface
    // TODO: Sim operator interface
    operatorInterface = new OperatorInterface(robotState);

    configureAutoModes();
    configureBindings();
  }

  private void configureAutoModes() {}

  private void configureBindings() {
    swerve.setDefaultCommand(swerve.joystickDrive(operatorInterface.getSwerveControlSignal()));
  }

  public Command getAutonomousCommand() {
    return operatorInterface.getAutoCommand();
  }
}
