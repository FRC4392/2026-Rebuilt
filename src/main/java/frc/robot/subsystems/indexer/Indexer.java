// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeceiverRobotState;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

  private final IndexerIO indexerIO;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  @SuppressWarnings("unused")
  private final DeceiverRobotState robotState;

  private final Alert indexerMotorDisconnectedAlert =
      new Alert("Feeder Motor Disconnected, may not be able to feed balls", AlertType.kError);

  /** Creates a new Indexer. */
  public Indexer(IndexerIO IO) {
    indexerIO = IO;
    robotState = DeceiverRobotState.getInstance();
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    indexerMotorDisconnectedAlert.set(!inputs.motorConnected);
  }

  public void setVoltage(Voltage volts) {
    indexerIO.setVoltage(volts);
  }

  public Command runTestVoltage() {
    return this.runEnd(() -> setVoltage(Volts.of(10)), () -> setVoltage(Volts.of(0)));
  }
}
