// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

  private final HopperIO hopperIO;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  /** Creates a new Hopper. */
  public Hopper(HopperIO IO) {
    hopperIO = IO;
  }

  @Override
  public void periodic() {
    hopperIO.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }

  public void setVoltage(Voltage volts) {
    hopperIO.setVoltage(volts);
  }

  public Command runTestVoltage() {
    return this.runEnd(() -> setVoltage(Volts.of(6)), () -> setVoltage(Volts.of(0)));
  }
}
