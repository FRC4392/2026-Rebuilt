// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.extensionTestVoltage;
import static frc.robot.subsystems.intake.IntakeConstants.intakeVoltage;
import static frc.robot.subsystems.intake.IntakeConstants.outtakeVoltage;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeceiverRobotState;
import frc.robot.DeceiverRobotState.IntakeExtensionStatus;
import frc.robot.DeceiverRobotState.IntakeRollerStatus;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final DeceiverRobotState robotState;

  private final Alert leftIntakeRollerAlert =
      new Alert(
          "Left intake roller motor not connected, performance my be reduced", AlertType.kError);
  private final Alert rightIntakeRollerAlert =
      new Alert(
          "Right intake roller motor not connected, performance my be reduced", AlertType.kError);
  private final Alert intakeExtensionAlert =
      new Alert("Intake extension motor not connected, inake may not function", AlertType.kError);

  /**
   * Creates new intake subsystem
   *
   * @param IO IO interface for intake
   * @param state Robot state to track different robot states
   */
  public Intake(IntakeIO IO, DeceiverRobotState state) {
    intakeIO = IO;
    robotState = state;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Log calculations
    Logger.recordOutput(
        "Intake/extensionPosition", extensionAngleToDistance(inputs.extensionMotorPosition));
    Logger.recordOutput(
        "Intake/rollerTorque",
        NewtonMeters.of((inputs.leftRollerMotorCurrent.in(Amps) * 0.01981) * rollerMotorReduction));

    // Connection checks
    intakeExtensionAlert.set(!inputs.extensionMotorConnected);
    leftIntakeRollerAlert.set(!inputs.leftRollerMotorConnected);
    rightIntakeRollerAlert.set(!inputs.rightRollerMotorConnected);

    // Calculate Roller Status
    if (inputs.leftRollerMotorAppliedVolts.isNear(intakeVoltage, Volts.of(2))) {
      robotState.setIntakeRolerStatus(IntakeRollerStatus.Intaking);
    } else if (inputs.leftRollerMotorAppliedVolts.isNear(outtakeVoltage, Volts.of(2))) {
      robotState.setIntakeRolerStatus(IntakeRollerStatus.Outtaking);
    } else if (inputs.leftRollerMotorAppliedVolts.isNear(Volts.of(0), Volts.of(2))) {
      robotState.setIntakeRolerStatus(IntakeRollerStatus.Stopped);
    } else {
      robotState.setIntakeRolerStatus(IntakeRollerStatus.UNKOWN);
    }

    // Calculate Extension Status
    if (inputs.extensionMotorPosition.isNear(
        extensionDistanceToAngle(extendedDistance), extensionDistanceToAngle(Inches.of(1)))) {
      robotState.setIntakeExtensionStatus(IntakeExtensionStatus.Extended);
    } else if (inputs.extensionMotorPosition.isNear(
        extensionDistanceToAngle(stowedDistance), extensionDistanceToAngle(Inches.of(1)))) {
      robotState.setIntakeExtensionStatus(IntakeExtensionStatus.Stowed);
    } else if (inputs.extensionMotorAppliedVolts.gt(Volts.of(2))) {
      robotState.setIntakeExtensionStatus(IntakeExtensionStatus.MovingExtend);
    } else if (inputs.extensionMotorAppliedVolts.lt(Volts.of(2))) {
      robotState.setIntakeExtensionStatus(IntakeExtensionStatus.MovingStowed);
    } else {
      robotState.setIntakeExtensionStatus(IntakeExtensionStatus.UNKOWN);
    }
  }

  // Conversions
  /**
   * Convert hopper extension distance to drive angle
   *
   * @param distance Distance of the hopper extension with 0 being all the way retracted
   * @return Corresponding drive angle to acheive that distance
   */
  private Angle extensionDistanceToAngle(Distance distance) {
    return Radians.of(distance.in(Meters) / (extensionDriveDiameter.in(Meters) / 2.0));
  }

  /**
   * Convert drive angle to extension distance
   *
   * @param angle Angle of the drive gear
   * @return Corresponding distance to based on that dive angle
   */
  private Distance extensionAngleToDistance(Angle angle) {
    return Meters.of((extensionDriveDiameter.in(Meters) / 2.0) * angle.in(Radians));
  }

  // Helper Functions
  /** Stop the intake roller */
  private void rollerStop() {
    intakeIO.setRoller(Volts.of(0));
  }

  /** Stop the intake extension */
  private void extensionStop() {
    intakeIO.setExtension(Volts.of(0));
  }

  /** Stop all intake mechanisms */
  private void stop() {
    rollerStop();
    extensionStop();
  }

  // Commands
  /**
   * Command to run the roller for intaking game pieces
   *
   * @return Command to run the roller for intaking game pieces
   */
  public Command runRollerIntake() {
    return this.runEnd(
        () -> {
          intakeIO.setRoller(intakeVoltage);
        },
        this::rollerStop);
  }

  /**
   * Command to run the roller for outtaking game pieces
   *
   * @return Command to run the roller for outtaking game pieces
   */
  public Command runRollerOuttake() {
    return this.runEnd(
        () -> {
          intakeIO.setRoller(outtakeVoltage);
        },
        this::rollerStop);
  }

  /**
   * Command to run the intake extension out of the robot manually The extension runs at a fixed
   * voltage and stops when the command is interrupted
   *
   * @return Command to run the intake extension out of the robot manually
   */
  public Command runExtensionOutManual() {
    return this.runEnd(
        () -> {
          intakeIO.setExtension(extensionTestVoltage);
        },
        this::extensionStop);
  }

  /**
   * Command to run the intake extension in to the robot manually The extension runs at a fixed
   * voltage and stops when the command is interrupted
   *
   * @return Command to run the intake extension in to the robot manually
   */
  public Command runExtensionInManual() {
    return this.runEnd(
        () -> {
          intakeIO.setExtension(extensionTestVoltage.unaryMinus());
        },
        this::extensionStop);
  }

  /**
   * Command to stop all motion of the intake
   *
   * @return Command to stop all motion of the intake
   */
  public Command stopAll() {
    return this.run(this::stop);
  }

  // TODO: Command for extending the intake automatically
  // TODO: Command for retracting the intake automatically
  // TODO: Command for normal game play, hold intake out, run intake and or stop intake, may be
  // multiple commands or command sequences
}
