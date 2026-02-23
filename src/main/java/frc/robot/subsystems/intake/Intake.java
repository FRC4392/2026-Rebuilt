// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intake.IntakeConstants.extensionTestVoltage;
import static frc.robot.subsystems.intake.IntakeConstants.intakeVoltage;
import static frc.robot.subsystems.intake.IntakeConstants.outtakeVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeceiverRobotState;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final DeceiverRobotState robotState;

  /** Creates a new Intake. */
  public Intake(IntakeIO IO, DeceiverRobotState state) {
    intakeIO = IO;
    robotState = state;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  private void rollerStop() {
    intakeIO.setRoller(Volts.of(0));
  }

  private void extensionStop() {
    intakeIO.setExtension(Volts.of(0));
  }

  public Command runRollerIntake() {
    return this.runEnd(
        () -> {
          intakeIO.setRoller(intakeVoltage);
        },
        this::rollerStop);
  }

  public Command runRollerOuttake() {
    return this.runEnd(
        () -> {
          intakeIO.setRoller(outtakeVoltage);
        },
        this::rollerStop);
  }

  public Command runExtensionOutManual() {
    return this.runEnd(
        () -> {
          intakeIO.setExtension(extensionTestVoltage);
        },
        this::extensionStop);
  }

  public Command runExtensionInManual() {
    return this.runEnd(
        () -> {
          intakeIO.setExtension(extensionTestVoltage.unaryMinus());
        },
        this::extensionStop);
  }
}
