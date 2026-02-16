// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  /** Creates a new Shooter. */
  public Shooter(ShooterIO IO) {
    shooterIO = IO;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void setShooter(Voltage volts) {
    shooterIO.setShooter(volts);
  }

  public Command runTestVoltage() {
    return this.runEnd(() -> setShooter(Volts.of(12)), () -> setShooter(Volts.of(0)));
  }

  public Command runTurret(DoubleSupplier voltage) {
    return this.runEnd(
        () -> setShooter(Volts.of(voltage.getAsDouble())), () -> setShooter(Volts.of(0)));
  }
}
