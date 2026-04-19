// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeceiverRobotState;
import frc.robot.DeceiverRobotState.FeederStatus;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

  private final HopperIO hopperIO;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  @SuppressWarnings("unused")
  private final DeceiverRobotState robotState;

  private final Alert bottomHopperMotorDisconnectedAlert =
      new Alert(
          "Bottom Hopper Roller Disconected, indexing ablility may be diminished",
          AlertType.kError);
  private final Alert topHopperMotorDisconnectedAlert =
      new Alert(
          "Top Hopper Roller Disconected, indexing ablility may be diminished", AlertType.kError);

  /** Creates a new Hopper. */
  public Hopper(HopperIO IO) {
    hopperIO = IO;
    robotState = DeceiverRobotState.getInstance();
  }

  @Override
  public void periodic() {
    hopperIO.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);

    topHopperMotorDisconnectedAlert.set(!inputs.topMotorConnected);
    bottomHopperMotorDisconnectedAlert.set(!inputs.bottomMotorConnected);

    if (inputs.bottomMotorAppliedVolts.gt(Volts.of(2))) {
      robotState.setFeederStatus(FeederStatus.Feeding);
    } else {
      robotState.setFeederStatus(FeederStatus.Stopped);
    }
  }

  public void setVoltage(Voltage volts) {
    hopperIO.setVoltage(volts);
  }

  public Command runTestVoltage() {
    return this.runEnd(() -> setVoltage(Volts.of(10)), () -> setVoltage(Volts.of(0)));
  }

  public Command vomit() {
    return this.runEnd(() -> setVoltage(Volts.of(-10)), () -> setVoltage(Volts.of(0)));
  }

  public AbsoluteEncoder getTurretAbsoluteEncoder() {
    return hopperIO.getTurretAbsoluteEncoder();
  }
}
