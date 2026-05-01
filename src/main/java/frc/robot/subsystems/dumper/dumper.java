// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.dumper;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeceiverRobotState;
import org.littletonrobotics.junction.Logger;

public class Dumper extends SubsystemBase {

  private final DumperIO dumperIO;
  private final DumperIOInputsAutoLogged inputs = new DumperIOInputsAutoLogged();
  private final DeceiverRobotState robotState;

  @SuppressWarnings("unused")
  private final Alert dumperMotorDisconnectedAlert = new Alert("Dumpa no dump", AlertType.kError);

  /** Creates a new Dumper. */
  public Dumper(DumperIO IO) {
    dumperIO = IO;
    robotState = DeceiverRobotState.getInstance();
  }

  @Override
  public void periodic() {
    dumperIO.updateInputs(inputs);
    Logger.processInputs("Dumper", inputs);

    dumperMotorDisconnectedAlert.set(!inputs.dumperMotor1Connected);
  }

  public void setVoltage(Voltage volts) {
    dumperIO.setVoltage(volts);
  }

  public Command runTestVoltage() {
    return this.runEnd(() -> setVoltage(Volts.of(6)), () -> setVoltage(Volts.of(0)));
  }
}
