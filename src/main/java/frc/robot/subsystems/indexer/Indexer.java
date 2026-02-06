// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

  private final IndexerIO indexerIO;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  /** Creates a new Indexer. */
  public Indexer(IndexerIO IO) {
    indexerIO = IO;
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  public void setVoltage(Voltage volts) {
    indexerIO.setVoltage(volts);
  }

  public Command runTestVoltage() {
    return this.runEnd(() -> setVoltage(Volts.of(6)), () -> setVoltage(Volts.of(0)));
  }
}
